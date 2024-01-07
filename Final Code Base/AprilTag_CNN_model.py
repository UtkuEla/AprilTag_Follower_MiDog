"""
This script is prepared for training the CNN model in Google Colab. It will not run as a .py, however it is delivered to show the work done.
Most of the functions in the script is prepared and used beforehand for different CNN projects. They may or may not be called. 
Plenty of CNN structures were trained for detecting the AprilTag locations. A model with small size and bearable accuracy is selected and saved in the end. 

For further inquiries about these scripts and CNN model please contact: utku.elagoez@tuhh.de
"""

import os
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tensorflow as tf
print("TensorFlow version:", tf.__version__)
from tensorflow import keras
from tensorflow.keras.layers import Dense, Flatten, Conv2D, MaxPooling2D, Dropout, Rescaling
from tensorflow.keras import Model

# Linking the Google Drive
from google.colab import drive
drive.mount("/content/drive")

%cd /content/drive/My\ Drive/AprilTagCNN
!ls

data_path = "/content/drive/MyDrive/AprilTagCNN/images/"
data_dir = Path(data_path) 
[print("Total ." + d + " images count= ", len(list(data_dir.glob('**/*.'+ d)))) for d in ["jpg", "jpeg", "png"]]
train_dataset_dir = Path(str(data_path  + "train/"))
test_dataset_dir = Path(str(data_path  + "test/"))

batch_size = 32
img_height = 256
img_width = 256
image_shape = (img_height, img_width)
input_shape = image_shape + tuple([3])

num_epochs = 50 # different epochs between 50-100 are tested.

# Use 80% of the images for training and 20% for validation.
train_val_split = 0.20 

train_ds = tf.keras.utils.image_dataset_from_directory(
    train_dataset_dir,
    validation_split=train_val_split,
    subset="training",
    seed=123,
    image_size=(img_height, img_width),
    batch_size=batch_size
    )
print("--> Training dataset is created.\n")

from keras.preprocessing.image import ImageDataGenerator

augmentation = ImageDataGenerator(rotation_range=15, zoom_range=0.1, width_shift_range=0.15, height_shift_range=0.15, brightness_range=(0.1, 1.0), fill_mode= "nearest", shear_range=0.15)

train_generator = augmentation.flow_from_directory(train_dataset_dir, target_size=image_shape, shuffle=False, batch_size=batch_size)

val_ds = tf.keras.utils.image_dataset_from_directory(
    train_dataset_dir,
    validation_split=train_val_split,
    subset="validation",
    seed=123,
    image_size=(img_height, img_width),
    batch_size=batch_size
    )
print("--> Validation dataset is created.\n")

test_ds = tf.keras.preprocessing.image_dataset_from_directory(
    test_dataset_dir, image_size=image_shape, 
    shuffle=False, batch_size=batch_size
    )
print("--> Test dataset is created.")

data = {}
for root, dirs, files in os.walk(str(train_dataset_dir), topdown=False):
    for dir in dirs:
        for r, d, f in os.walk(str(train_dataset_dir) + '/'+ dir, topdown=False):
            data[dir] = str(train_dataset_dir) + '/'+ dir + '/' + f[0]

data = dict(sorted(data.items()))
print(data)

AUTOTUNE = tf.data.AUTOTUNE

train_ds = train_ds.cache().shuffle(1000).prefetch(buffer_size=AUTOTUNE)
val_ds = val_ds.cache().prefetch(buffer_size=AUTOTUNE)

def visualize(history, epochs, title):
    acc = history.history['accuracy']
    val_acc = history.history['val_accuracy']

    loss = history.history['loss']
    val_loss = history.history['val_loss']

    epochs_range = range(epochs)

    plt.figure(figsize=(8, 8))
    plt.subplot(1, 2, 1)
    plt.plot(epochs_range, acc, label='Training Accuracy')
    plt.plot(epochs_range, val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title(title + ' Accuracy')

    plt.subplot(1, 2, 2)
    plt.plot(epochs_range, loss, label='Training Loss')
    plt.plot(epochs_range, val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.title(title + ' Loss')
    plt.show()

def createModel():
    #one of the example models with a basic setting to detect the AprilTags.
    model = Sequential()
    model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=(256, 256, 3)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Flatten())
    model.add(Dense(256, activation='relu'))
    model.add(Dense(1, activation='linear'))
    
    return model 


model = createModel()

model.summary()

model.compile(loss='mean_squared_error', optimizer='adam')

model.save('model.h5')

metrics1 = model.fit(train_ds, epochs=num_epochs, validation_data=val_ds)

pd.options.display.max_rows = 4000
tf.keras.utils.plot_model(model,'model.png', show_shapes=True)

visualize(metrics1, num_epochs, "Approch 1 -")