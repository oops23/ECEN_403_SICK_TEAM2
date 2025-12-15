import os
import csv

# Base folder containing subfolders for each insect species
base_folder = "/scratch/user/jmagnew/datasets/insects"

# Output CSV file which will store image paths and labels
output_csv = "insect_labels.csv"

# Dictionary mapping each species folder to its role (Pollinator or Pest)
species_role_map = {
    "Bee": "Pollinator",
    "Butterfly": "Pollinator",
    "Dragonfly": "Pollinator",
    "Ladybug": "Pollinator",
    "Beetle": "Pest",
    "Grasshopper": "Pest",
    "Spider": "Pest",
    "Wasp": "Pest",
}

# Open the CSV file for writing
with open(output_csv, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.writer(csvfile)
    # Write the header row
    writer.writerow(["image_path", "species", "role"])

    # Iterate through each subfolder (species) in the base folder
    for species in os.listdir(base_folder):
        species_folder = os.path.join(base_folder, species)
        # Skip if not a directory (in case extra files exist)
        if not os.path.isdir(species_folder):
            continue

        # Assign role using the species_role_map
        role = species_role_map.get(species, "Unknown")

        # Iterate through each image file in the species folder
        for img_file in os.listdir(species_folder):
            # Only proceed for image files with allowed extensions
            if not img_file.lower().endswith((".jpg", ".jpeg", ".png")):
                continue
            # Get the full image path
            img_path = os.path.join(species_folder, img_file)
            # Write a row to CSV: [image path, species, role]
            writer.writerow([img_path, species, role])

print(f"Annotation CSV written to {output_csv}")