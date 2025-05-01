from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
from PIL import Image
import numpy as np
import cv2
from matplotlib import pyplot as plt

# Load the model and processor
processor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b0-finetuned-ade-512-512")
model = SegformerForSemanticSegmentation.from_pretrained("nvidia/segformer-b0-finetuned-ade-512-512")

# Get the class labels from the model's configuration
class_labels = model.config.id2label

# Create a color map for visualization
num_classes = len(class_labels)
colors = np.array(plt.cm.get_cmap("tab20", num_classes).colors)[:, :3] * 255  # RGB colors
print("Colors shape:", class_labels)
# Open the camera stream
cap = cv2.VideoCapture(1)  # Use 0 for the default camera

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the frame to PIL Image
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Preprocess the image
    inputs = processor(images=image, return_tensors="pt")
    outputs = model(**inputs)
    logits = outputs.logits  # shape (batch_size, num_labels, height/4, width/4)
    predicted_semantic_map = logits.argmax(dim=1).squeeze().numpy()  # get the predicted labels for each pixel

    uqeue_labels = np.unique(predicted_semantic_map)
    print("Unique labels in predicted map:", uqeue_labels)

    # Create a color image for the segmentation map
    segmented_image = np.zeros((predicted_semantic_map.shape[0], predicted_semantic_map.shape[1], 3), dtype=np.uint8)
    for class_id, color in enumerate(colors):
        segmented_image[predicted_semantic_map == class_id] = color.astype(np.uint8)

    # Resize the segmented image to match the original frame size
    segmented_image = cv2.resize(segmented_image, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

    # Overlay the segmentation on the original frame
    overlay = cv2.addWeighted(frame, 0.5, segmented_image, 0.5, 0)

    # Add class labels to the overlay
    for class_id, label in class_labels.items():
        mask = (predicted_semantic_map == class_id)
        y_coords, x_coords = np.where(mask)
        if len(x_coords) > 0 and len(y_coords) > 0:
            # Compute scaling factors from predicted map size to frame size
            scale_x = frame.shape[1] / predicted_semantic_map.shape[1]
            scale_y = frame.shape[0] / predicted_semantic_map.shape[0]
            # Calculate the centroid on the low-res map and upscale to the original frame
            centroid_x = int(np.mean(x_coords) * scale_x)
            centroid_y = int(np.mean(y_coords) * scale_y)
            cv2.putText(overlay, label, (centroid_x, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # Display the overlay
    cv2.imshow("Segmentation", overlay)
    # cv2.imshow("Segmented Image", segmented_image)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
