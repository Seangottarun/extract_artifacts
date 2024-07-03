# Open the file detected_artifacts.csv
# Create new csv file called filtered_artifacts.csv

import csv
import math

def distance(p1, p2):
    """Calculate the Euclidean distance between two points in 3D space."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

def filter_artifacts(input_file, output_file, tolerance):
    # Read the detected artifacts from the input CSV file
    with open(input_file, mode='r') as infile:
        reader = csv.reader(infile)
        detected_artifacts = list(reader)

    # Initialize a list to hold the filtered artifacts
    filtered_artifacts = []

    # Iterate over each detected artifact
    for artifact in detected_artifacts:
        class_id, x, y, z = artifact[0], float(artifact[1]), float(artifact[2]), float(artifact[3])
        artifact_position = (x, y, z)

        # Check if the class_id is already in the filtered list
        class_id_exists = False
        for existing_artifact in filtered_artifacts:
            existing_class_id, ex, ey, ez = existing_artifact[0], float(existing_artifact[1]), float(existing_artifact[2]), float(existing_artifact[3])
            existing_position = (ex, ey, ez)

            if existing_class_id == class_id:
                class_id_exists = True
                if distance(artifact_position, existing_position) > tolerance:
                    filtered_artifacts.append(artifact)
                break

        # If class_id does not exist in filtered_artifacts, add the artifact
        if not class_id_exists:
            filtered_artifacts.append(artifact)

    # Write the filtered artifacts to the output CSV file
    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerows(filtered_artifacts)

if __name__ == '__main__':
    input_file = './src/extract_artifacts/detected_artifacts/detected_artifacts.csv'
    output_file = './src/extract_artifacts/detected_artifacts/filtered_artifacts.csv'
    tolerance = 10.0

    filter_artifacts(input_file, output_file, tolerance)
