import os
import yaml

def load_yaml_files(folder_path):
    folder_path = os.path.expanduser(folder_path)

    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Folder not found: {folder_path}")

    all_data = {}
    # Iterate over all YAML files in the folder
    # Preferably keep only 1 yaml in such a folder to avoid confusion
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.yaml') or file_name.endswith('.yml'):
            file_path = os.path.join(folder_path, file_name)
            with open(file_path, 'r') as file:
                try:
                    data = yaml.safe_load(file)
                    all_data[file_name] = data  # Use file name as the key
                except yaml.YAMLError as e:
                    print(f"Error loading YAML file {file_name}: {e}")

    return all_data


def main(args=None):
    folder_path = "~Desktop/ros2_ws/src/Robosub_ROS2_2024/auv_mission_control/config"

    try:
        yaml_data = load_yaml_files(folder_path)
        print(f"Successfully loaded YAML data:\n{yaml_data}")
    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    import sys
    main(sys.argv)

