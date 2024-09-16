import os
import xml.etree.ElementTree as ET

urdf_name = "h1_with_hand.urdf"
urdf_path = os.path.join(os.path.dirname(__file__), f'../urdf/{urdf_name}')

def get_joint_names(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joint_names = [joint.get('name') for joint in root.findall('.//joint') if joint.get('type') != 'fixed']
    return joint_names

if __name__ == '__main__':
    joint_names = get_joint_names(urdf_path)
    print(joint_names, len(joint_names))