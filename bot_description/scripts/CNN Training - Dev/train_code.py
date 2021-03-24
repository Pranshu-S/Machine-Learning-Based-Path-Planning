import numpy as np
import cv2
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Activation, Dense, Flatten, BatchNormalization, Conv2D, MaxPool2D, Dropout
from tensorflow.keras.optimizers import Adam
from keras.utils.np_utils import to_categorical
from tensorflow.keras.metrics import categorical_crossentropy
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import train_test_split
import itertools
import os
import shutil
import random
import glob
import pickle
import pandas as pd
import matplotlib.pyplot as plt
import warnings
warnings.simplefilter(action='ignore',category=FutureWarning)
# %matplotlib inline

train_path='/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/training/data/train'
test_path='/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/training/data/test'
valid_path='/home/pranshu/ROS_WORKSPACES/IEEE_P1/src/bot_description/training/data/validation'

train_batches=ImageDataGenerator().flow_from_directory(train_path, target_size=(224,224), classes=['0','1','2','3'], batch_size=10)
test_batches=ImageDataGenerator().flow_from_directory(test_path, target_size=(224,224), classes=['0','1','2','3'], batch_size=10)
valid_batches=ImageDataGenerator().flow_from_directory(valid_path, target_size=(224,224), classes=['0','1','2','3'], batch_size=10)


def plots(ims, figsize=(12,6), rows=1, interp=False, titles=None):
    if type(ims[0]) is np.ndarray:
        ims = np.array(ims).astype(np.uint8)
        if (ims.shape[-1] != 3):
            ims = ims.transpose((0,2,3,1))
    f = plt.figure(figsize=figsize)
    cols = len(ims)//rows if len(ims) % 2 == 0 else len(ims)//rows + 1
    for i in range(len(ims)):
        sp = f.add_subplot(rows, cols, i+1)
        sp.axis('Off')
        if titles is not None:
            sp.set_title(titles[i], fontsize=16)
        plt.imshow(ims[i], interpolation=None if interp else 'none')
        

imgs, labels = next(train_batches)
plots(imgs, titles=labels)
print(imgs.shape,'\n')

batch_size_val=50
steps_per_epoch_val=2000
epochs_val=30

def myModel():
    no_of_Filters=60
    size_of_Filter=(5,5)
    size_of_Filter2=(3,3)
    size_of_pool=(2,2)
    no_of_Nodes=500
    model=Sequential()
    model.add((Conv2D(no_of_Filters,size_of_Filter,input_shape=(224,224,3),activation='relu')))
    model.add((Conv2D(no_of_Filters,size_of_Filter,activation='relu')))
    model.add(MaxPool2D(pool_size=size_of_pool))
    
    model.add((Conv2D(no_of_Filters,size_of_Filter2,activation='relu')))
    model.add((Conv2D(no_of_Filters,size_of_Filter2,activation='relu')))
    model.add(MaxPool2D(pool_size=size_of_pool))
    model.add(Dropout(0.5))
    
    model.add(Flatten())
    model.add(Dense(no_of_Nodes,activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(4,activation='softmax'))
        
    model.compile(optimizer=Adam(lr=0.0001), loss='categorical_crossentropy',metrics=['accuracy'])
    return model

model=myModel()
model.summary()
history=model.fit_generator(train_batches, steps_per_epoch=30, validation_data=valid_batches, validation_steps=4, epochs=5, verbose=1)

test_imgs, test_labels = next(test_batches)
plots(test_imgs, titles=test_labels)

predictions = model.predict_generator(test_batches, steps=1, verbose=0)

model.save('/home/ros/IEEE/src/bot_description/CNN/CNN.h5')