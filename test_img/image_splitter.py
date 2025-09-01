from PIL import Image
import os
import os.path

def split_image_into_4(image_path: str, output_folder: str = 'output_img'):
    # Load the image
    image = Image.open(image_path)
    width, height = image.size

    # Calculate the size of each quadrant
    mid_width = width // 2
    mid_height = height // 2

    # Ensure the output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # Extract the base filename without extension
    base_name = os.path.splitext(os.path.basename(image_path))[0]

    # Define the coordinates of each quadrant
    quadrants = [
        (0, 0, mid_width, mid_height),  # Top-left
        (mid_width, 0, width, mid_height),  # Top-right
        (0, mid_height, mid_width, height),  # Bottom-left
        (mid_width, mid_height, width, height)  # Bottom-right
    ]

    # Crop and save each quadrant
    for i, coords in enumerate(quadrants):
        cropped_image = image.crop(coords)
        output_path = os.path.join(output_folder, f'{base_name}_{i + 1}.jpg')
        cropped_image.save(output_path)
        print(f'Saved {output_path}')

# Example usage
split_image_into_4('test_car.jpg')

