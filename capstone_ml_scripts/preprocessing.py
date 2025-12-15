# preprocessing.py
import pandas as pd
from sklearn.model_selection import train_test_split

def get_preprocessed_df(csv_path='insect_labels.csv'):
    # Load the CSV file containing image paths and labels
    df = pd.read_csv(csv_path)
    
    # Get the list of unique species and ensure consistent order
    species_list = sorted(df['species'].unique())
    # Build a mapping from species name to numeric index
    species_to_idx = {s: i for i, s in enumerate(species_list)}
    # Create a new column with integer-encoded species labels
    df['species_idx'] = df['species'].map(species_to_idx)
    # Map roles to 1 for Pollinator, 0 for Pest
    df['role_idx'] = df['role'].map({'Pollinator': 1, 'Pest': 0})
    
    # Drop any rows missing required info (image path, encoded species or role)
    df = df.dropna(subset=['image_path', 'species_idx', 'role_idx'])
    
    # Split data into train, validation, and test sets (stratified by species)
    train_df, test_df = train_test_split(
        df, test_size=0.1, stratify=df['species_idx'], random_state=42)
    train_df, val_df = train_test_split(
        train_df, test_size=0.1, stratify=train_df['species_idx'], random_state=42)
    
    # Print checks for label balance and encoding
    print(df['role'].value_counts())         # Print counts for 'Pollinator' and 'Pest'
    print(set(df['role_idx']))               # Should print {0, 1}
    print(df[['species', 'role']].head(20))  # Show first 20 rows of species and role

    # Return processed train, validation, test sets, and species list for modeling
    return train_df, val_df, test_df, species_list

if __name__ == '__main__':
    train_df, val_df, test_df, species_list = get_preprocessed_df()
    print(species_list)  # Print summary of species in your dataset