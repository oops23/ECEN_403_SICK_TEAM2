# model.py
import tensorflow as tf

def make_model(num_species, img_size=(224, 224)):
    """
    Builds and compiles a MobileNetV2-based multi-output model for:
      1. Species classification (multiclass, softmax)
      2. Role classification (binary: pollinator/pest, sigmoid)

    Args:
        num_species (int): Number of unique insect species/classes.
        img_size (tuple): Image width and height.

    Returns:
        tf.keras.Model: Compiled multi-output model ready for training.
    """

    # Load base MobileNetV2 model pre-trained on ImageNet, without top classifier layers
    base = tf.keras.applications.MobileNetV2(
        input_shape=img_size + (3,),
        include_top=False,
        weights='imagenet'
    )
    # Freeze the base model's weights so only top layers are trainable
    base.trainable = False

    # Input for images of shape (224,224,3) by default
    inputs = tf.keras.Input(shape=img_size + (3,))

    # Extract features using the (frozen) base model
    x = base(inputs, training=False)
    # Condense spatial dimensions to a single vector
    x = tf.keras.layers.GlobalAveragePooling2D()(x)

    # Classification head for species (multi-class, softmax)
    species_out = tf.keras.layers.Dense(
        num_species, activation='softmax', name='species')(x)

    # Classification head for role (binary, sigmoid)
    role_out = tf.keras.layers.Dense(
        1, activation='sigmoid', name='role')(x)

    # Create the multi-output model
    model = tf.keras.Model(inputs, [species_out, role_out])

    # Compile the model with optimizer, losses, and accuracy metrics for both outputs
    model.compile(
        optimizer='adam',
        loss={
            'species': 'categorical_crossentropy',
            'role': 'binary_crossentropy'
        },
        metrics={
            'species': 'accuracy',
            'role': 'accuracy'
        }
    )
    return model