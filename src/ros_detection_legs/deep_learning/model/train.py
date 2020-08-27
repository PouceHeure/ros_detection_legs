import os 
import csv
import numpy as np 

from sklearn.model_selection import train_test_split
import tensorflow as tf 
from tensorflow import keras 

PATH_FILE_CURRENT = os.path.dirname(os.path.realpath(__file__))
PATH_FOLDER_DATA = os.path.join(PATH_FILE_CURRENT,"../../../../data/")
PATH_FOLDER_DATA_PROCESSED = os.path.join(PATH_FOLDER_DATA,"processed/")


def load_data(file_path,length_fill = 30): 
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
    X_train_full, y_train_full = load_data(os.path.join(PATH_FOLDER_DATA_PROCESSED,"train.csv"))
    X_train, X_valid, y_train, y_valid = train_test_split(X_train_full,y_train_full)

    model = keras.models.Sequential([
        keras.layers.RNN(keras.layers.LSTMCell(10),return_sequences=True,input_shape=[None,2]),
        keras.layers.RNN(keras.layers.LSTMCell(10)),
        keras.layers.Dense(1,activation="sigmoid")
    ])

    model.compile(loss="binary_crossentropy",optimizer="adam",metrics=["accuracy"])
    model.fit(X_train,y_train,validation_data=(X_valid,y_valid),epochs=100)

    model.save(os.path.join(PATH_FILE_CURRENT,"../../model","train"))

    index_select = 0 
    for i in range(y_valid.shape[0]): 
        if(y_valid[i] == 1): 
            index_select = i 
            break 

    X_test = np.array(X_valid[index_select:index_select+10])
    print(y_valid[index_select:index_select+10])
    print(model.predict(X_test))


