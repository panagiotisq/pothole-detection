import cv2
import os

def make_video_from_images_opencv(img_folder, output_video, fps=10):
    files = sorted([f for f in os.listdir(img_folder) if f.endswith(".png")])
    if not files:
        print("No images found!")
        return

    # Get size from first image
    first_frame = cv2.imread(os.path.join(img_folder, files[0]))
    height, width, _ = first_frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # .mp4 codec
    out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for f in files:
        img_path = os.path.join(img_folder, f)
        frame = cv2.imread(img_path)
        out.write(frame)

    out.release()
    print(f"Video saved as {output_video}")

if __name__ == "__main__":
    img_folder = "screenshots"
    output_video = "output.mp4"
    make_video_from_images_opencv(img_folder, output_video)
