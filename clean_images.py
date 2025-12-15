from PIL import Image
import pandas as pd
import os

# Input and output CSV file paths
orig_csv = 'insect_labels.csv'
out_csv = 'insect_labels_clean.csv'

# Load the original annotation CSV with image paths, species, and role
df = pd.read_csv(orig_csv)

good_rows = []  # Will hold only valid image rows

# Iterate through every row in the CSV
for path, species, role in zip(df['image_path'], df['species'], df['role']):
    # Check if the file exists and is not zero bytes
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        print("Missing or zero-byte file:", path)
        continue
    try:
        # Try to open the image to ensure it's a valid image file
        with Image.open(path) as img:
            img.verify()  # Will raise an error if the file is not a valid image
    except Exception as e:
        # If it's corrupt or invalid, skip it and print a warning
        print(f"Bad image skipped: {path} ({e})")
        continue
    # If the image is valid, add the row to the cleaned dataset
    good_rows.append((path, species, role))

# Create a cleaned DataFrame and save as a new CSV
df_clean = pd.DataFrame(good_rows, columns=['image_path', 'species', 'role'])
df_clean.to_csv(out_csv, index=False)
print(f"Cleaned CSV written to {out_csv} with {len(df_clean)} out of {len(df)} images.")