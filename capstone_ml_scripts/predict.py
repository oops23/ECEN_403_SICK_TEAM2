import numpy as np
from tensorflow.keras.models import load_model
import tensorflow as tf

# Your saved model (update path as needed)
model = load_model('my_insect_model.h5')

IMG_SIZE = (224, 224)

# Replace with the real image path you want to test
img_path = '/scratch/user/jmagnew/datasets/insects/Bee/Bee_1.jpg'

# Preprocessing function (as in training)
def preprocess_image(path):
    img = tf.io.read_file(path)
    img = tf.image.decode_jpeg(img, channels=3)  # if testing PNG, use decode_png!
    img = tf.image.resize(img, IMG_SIZE)
    img = img / 255.0  # Normalization
    img = tf.expand_dims(img, axis=0)  # Add batch dimension
    return img

pred_img = preprocess_image(img_path)

# Run prediction
species_pred, role_pred = model.predict(pred_img)

# Your list of species (should match training order!)
species_list = ['Bee', 'Beetle', 'Butterfly', 'Dragonfly', 'Grasshopper', 'Ladybug', 'Spider', 'Wasp']  # example

species_label = species_list[np.argmax(species_pred)]
role_label = 'Pollinator' if role_pred[0][0] > 0.5 else 'Pest'

print("Predicted species:", species_label, "| Role:", role_label)