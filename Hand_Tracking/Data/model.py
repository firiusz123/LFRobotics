from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.yaml")

# Train the model
result = model.train(model="./Data/data.yaml", epochs=3)
