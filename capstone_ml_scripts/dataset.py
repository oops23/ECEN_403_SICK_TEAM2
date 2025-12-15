# dataset.py
import tensorflow as tf

# Target image size for the model
IMG_SIZE = (224, 224)

def preprocess(path, species_idx, role_idx, num_species):
    """Load and preprocess a single image and its labels for model input.
    - Reads the image file, decodes JPEG, resizes, and normalizes.
    - Converts species index to one-hot and role index to float.
    """
    # Read image file
    img = tf.io.read_file(path)
    # Decode JPEG (3 color channels)
    img = tf.image.decode_jpeg(img, channels=3)
    # Resize to target dimensions
    img = tf.image.resize(img, IMG_SIZE)
    # Normalize pixel values to [0, 1]
    img = img / 255.0
    # One-hot encode species
    species_onehot = tf.one_hot(species_idx, num_species)
    # Ensure role label is float (for binary crossentropy)
    role_idx = tf.cast(role_idx, tf.float32)
    # Return processed image and labels as a tuple
    return img, (species_onehot, role_idx)

def df_to_dataset(dataframe, num_species, batch_size=32, shuffle=True):
    """Create a tf.data.Dataset from a dataframe with images and labels to produce batches of tensors."""
    # Extract file paths and labels from the dataframe
    paths = dataframe['image_path'].astype(str).values      # Image file paths
    species = dataframe['species_idx'].values               # Encoded species indices
    roles = dataframe['role_idx'].values                    # 0/1 for Pest/Pollinator
    # Create a dataset of (path, species_idx, role_idx) tuples
    ds = tf.data.Dataset.from_tensor_slices((paths, species, roles))
    # Map preprocessing to each dataset element
    ds = ds.map(lambda p, s, r: preprocess(p, s, r, num_species), num_parallel_calls=tf.data.AUTOTUNE)
    if shuffle:
        # Shuffle the dataset with a buffer equal to the dataset size.
        # This means each batch will be drawn randomly from the entire dataset,
        # improving training and generalization
        ds = ds.shuffle(buffer_size=len(paths))
    # Batch and prefetch for performance
    ds = ds.batch(batch_size).prefetch(tf.data.AUTOTUNE)
    return ds