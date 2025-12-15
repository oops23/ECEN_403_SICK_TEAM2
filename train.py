import preprocessing
import dataset
import model as model_module
import tensorflow as tf
import numpy as np
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score
import matplotlib.pyplot as plt
import seaborn as sns

log_dir = "logs/insect_model"
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1, write_images=True)

def main():
    # Step 1: Data prep
    train_df, val_df, test_df, species_list = preprocessing.get_preprocessed_df("insect_labels_clean.csv")
    num_species = len(species_list)
    # Step 2: Datasets
    train_ds = dataset.df_to_dataset(train_df, num_species)
    val_ds   = dataset.df_to_dataset(val_df, num_species, shuffle=False)
    test_ds  = dataset.df_to_dataset(test_df, num_species, shuffle=False)
    # Step 3: Model
    model = model_module.make_model(num_species)
    # Step 4: Train
    model.fit(
        train_ds,
        validation_data=val_ds,
        epochs=15,
        callbacks=[
            tf.keras.callbacks.EarlyStopping(patience=3, restore_best_weights=True),
            tensorboard_callback
        ]
    )
    # Step 5: Evaluate
    print("Model evaluation:")
    results = model.evaluate(test_ds)
    print(results)

    # -------- EXTENDED EVALUATION --------
    y_true_species, y_pred_species = [], []
    y_true_role, y_pred_role = [], []
    for batch in test_ds:
        images, (species_label, role_label) = batch
        # species_label: (batch, num_species) one-hot
        # role_label: (batch,)
        y_true_species.extend(np.argmax(species_label.numpy(), axis=1))
        y_true_role.extend(role_label.numpy().round().astype(int))
        # Forward pass
        species_pred, role_pred = model.predict(images)
        y_pred_species.extend(np.argmax(species_pred, axis=1))
        y_pred_role.extend((role_pred > 0.5).astype(int).reshape(-1))

    y_true_species = np.array(y_true_species)
    y_pred_species = np.array(y_pred_species)
    y_true_role = np.array(y_true_role)
    y_pred_role = np.array(y_pred_role)

    # --- Species stats ---
    cm_species = confusion_matrix(y_true_species, y_pred_species)
    acc_species = np.mean(y_true_species == y_pred_species)
    precision_species = precision_score(y_true_species, y_pred_species, average='macro', zero_division=0)
    recall_species    = recall_score(y_true_species, y_pred_species, average='macro', zero_division=0)
    f1_species        = f1_score(y_true_species, y_pred_species, average='macro', zero_division=0)
    per_class_acc = cm_species.diagonal() / cm_species.sum(axis=1)

    # --- Role stats (binary) ---
    acc_role = np.mean(y_true_role == y_pred_role)
    precision_role = precision_score(y_true_role, y_pred_role, average='binary')
    recall_role    = recall_score(y_true_role, y_pred_role, average='binary')
    f1_role        = f1_score(y_true_role, y_pred_role, average='binary')
    cm_role = confusion_matrix(y_true_role, y_pred_role)

    # Model parameter count
    total_params = model.count_params()
    trainable_params = np.sum([np.prod(v.shape) for v in model.trainable_weights])

    # Plot confusion matrix for species
    plt.figure(figsize=(10, 8))
    sns.heatmap(cm_species, annot=True, fmt="d", cmap="Blues",
                xticklabels=species_list, yticklabels=species_list)
    plt.xlabel("Predicted Species")
    plt.ylabel("True Species")
    plt.title("Confusion Matrix - Species")
    plt.tight_layout()
    plt.savefig("confusion_matrix_species.png", dpi=120)
    plt.show()

    # Also print confusion matrix for role
    print("Pollinator/Pest confusion matrix:\n", cm_role)

    # Print report
    print("\n========== Insect Model Statistics ==========")
    print(f"Number of species classes: {len(species_list)}")
    print(f"Input image size: 224x224x3")
    print(f"Test images evaluated: {len(y_true_species)}")
    print(f"Model parameter count: {total_params:,} (Trainable: {trainable_params:,})")
    print(f"Species Test Accuracy: {acc_species*100:.2f}%")
    print(f"Species Macro Precision:   {precision_species:.4f}")
    print(f"Species Macro Recall:      {recall_species:.4f}")
    print(f"Species Macro F1 Score:    {f1_species:.4f}")
    print("Species Per-Class Accuracy:")
    for i, cname in enumerate(species_list):
        print(f"  {cname:>10}: {per_class_acc[i]:.3f}")
    print("--- Role (Pollinator/Pest):")
    print(f"Role Test Accuracy: {acc_role*100:.2f}%")
    print(f"Role Precision:   {precision_role:.4f}")
    print(f"Role Recall:      {recall_role:.4f}")
    print(f"Role F1 Score:    {f1_role:.4f}")
    print("=============================================")

    # Step 6: Save
    model.save('my_insect_model2.h5')
    print("Model saved.")

if __name__ == "__main__":
    main()