import os 
import csv
import numpy as np 

# import deep learning 

from sklearn.model_selection import train_test_split
import tensorflow as tf 
from tensorflow import keras 
from ros_detection_legs.deep_learning.config import loader

# paths 

PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FOLDER_DATA = os.path.join(PATH_FILE_CURRENT,"../data/")
PATH_FOLDER_DATA_PROCESSED = os.path.join(PATH_FOLDER_DATA,"processed/")

def load_data_from_csv(file_path,length_fill = 30): 
    X = []
    y = []
    with open(file_path, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in spamreader:
            y.append(int(row[0]))
            points = []
            for point in row[1:]: 
                points.append(list(map(float,point.split("%"))))
            points += [[0,0]]*(length_fill-len(points)) #+ points
            X.append(points)
    return np.array(X),np.array(y,dtype=np.int) 
            
if __name__ == "__main__":
    config = loader.load_parameters()
    config_preprocessing = config["prepocessing"]
    config_training = config["training"]

    X_train_full, y_train_full = load_data_from_csv(os.path.join(PATH_FOLDER_DATA_PROCESSED,"train.csv")
                                                    ,length_fill=config_preprocessing["limit_length_data"])

    X_train, X_valid, y_train, y_valid = train_test_split(X_train_full,y_train_full)

    cb_early_stopping = tf.keras.callbacks.EarlyStopping(patience=config_training["patience"],restore_best_weights=True)

    model = keras.models.Sequential([
        keras.layers.RNN(keras.layers.LSTMCell(20),return_sequences=True,input_shape=[None,2]),
        keras.layers.RNN(keras.layers.LSTMCell(20)),
        keras.layers.Dense(1,activation="sigmoid")
    ])

    model.compile(loss="binary_crossentropy"
                ,optimizer="adam"
                ,metrics=["accuracy"])

    model.fit(X_train,y_train
            ,validation_data=(X_valid,y_valid)
            ,epochs=config_training["epochs"]
            ,batch_size=32
            ,callbacks=[cb_early_stopping])

    model.save(os.path.join(PATH_FILE_CURRENT,"../model","train"))

    index_select = 0
    length = 10
    X_test = np.array(X_valid[index_select:index_select+length])
    print(y_valid[index_select:index_select+length])
    print(model.predict(X_test))


