
from modules.segmentation import *  

def load_paths_files(folder_root):
    paths_files = []
    for file_name in os.listdir(folder_root): 
        file_path = os.path.join(folder_root, file_name)
        if os.path.isfile(file_path) and ".csv" in file_name:
            paths_files.append(file_path)
    return paths_files

def keep_proportion(X,y): 
    number_y_to_1 = y.count(1)
    new_X = []
    new_y = []
    number_y0_added = 0
    for i in range(len(X)): 
        if(len(X[i]) < 30):
            if(y[i] == 1 or (y[i] == 0 and number_y0_added < 6*number_y_to_1)):
                new_X.append(X[i])
                new_y.append(y[i]) 
                number_y0_added += 1-y[i] 

    return new_X,new_y

def create_dataset_train(folder_in,file_out):
    paths_files = load_paths_files(folder_in) 
    X = []
    y = []
    for path in paths_files: 
        filedata = LidarData(type="train")
        filedata.load_data_from_csv(path)
        filedata.processing(epsilon=0.5,gamma=0.8,limit_jump=20,limit_radius=0.30)
        X_, y_ = filedata.generate_dataset(type="train")
        X += X_
        y += y_

    X,y = keep_proportion(X,y)

    with open(file_out, 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            for i in range(len(X)): 
                spamwriter.writerow([y[i]] + X[i])


if __name__ == "__main__":  
    create_dataset_train(PATH_FOLDER_DATASET_LIDAR_50cm,os.path.join(PATH_FOLDER_DATA_PROCESSED,"train.csv"))