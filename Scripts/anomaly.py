import pandas as pd
import argparse
import os

def find_anomalies(input_file, output_file, threshold=30.0):
    """
    Scans a CSV wind report for anomalous wind speeds.
    
    Args:
        input_file (str): Path to the input CSV file.
        output_file (str): Path to save the anomalous rows.
        threshold (float): Wind speed threshold in m/s.
    """
    if not os.path.exists(input_file):
        print(f"Error: File '{input_file}' not found.")
        return

    try:
        # Load the data
        df = pd.read_csv(input_file)
        
        # Ensure the column exists
        if 'wind_speed_mean' not in df.columns:
            print("Error: Column 'wind_speed_mean' not found in CSV.")
            return

        # Filter: Find rows where wind speed exceeds the threshold
        # We also check for NaN values just in case
        anomalies = df[
            (df['wind_speed_mean'] > threshold) | 
            (df['wind_speed_mean'].isna())
        ]

        num_found = len(anomalies)

        if num_found > 0:
            anomalies.to_csv(output_file, index=False)
            print(f"FAILURE DETECTED: Found {num_found} anomalous rows.")
            print(f"Rows with wind_speed_mean > {threshold} m/s saved to '{output_file}'.")
        else:
            print(f"SUCCESS: No anomalies found (Threshold: {threshold} m/s).")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    # You can run this from command line: python script.py reports.csv
    # Or just hardcode the filenames below
    INPUT_FILENAME = 'reports.csv'  # Change this to your file name
    OUTPUT_FILENAME = 'anomalous_values.csv'
    
    find_anomalies(INPUT_FILENAME, OUTPUT_FILENAME)