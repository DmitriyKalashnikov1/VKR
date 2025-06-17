from ultralytics import YOLO
import roboflow
import shutil


needDownload = int(input("Need to download train dataset? [0 -- no, other -- yes]: "))

if not (needDownload == 0):
    roboflow.login()
    rf = roboflow.Roboflow()
    project =  rf.workspace("konkuk-university-9whom").project("cctv-person-detection")
    dataset = project.version(3).download("yolov11")
    shutil.copytree("./cctv-person-detection-3/", "./", dirs_exist_ok=True)
    shutil.rmtree("./cctv-person-detection-3")


# Load a model
model = YOLO("yolo11s.yaml")

# Train the model
train_results = model.train(
    data="./data.yaml",  # path to dataset YAML
    epochs=500,  # number of training epochs
    imgsz=640,  # training image size
    device="0",  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
)

# Evaluate model performance on the validation set
metrics = model.val()


# Export the model to NCNN format
path = model.export(format="NCNN")  # return path to exported model
