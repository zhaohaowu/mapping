import matplotlib.pyplot as plt
import argparse


def extract_data_from_log_file(intput_file):
    with open(intput_file, "r") as file:
        datas = file.readlines()
        datas = [data for data in datas if "[predict]" in data]
        # print(datas)
        
        block_idxs = [idx for idx, data in enumerate(datas) if "[predict] idx:" in data]
        print(block_idxs)
        block_datas = dict()
        for idx in range(len(block_idxs)-1):
            block_datas[idx] = datas[block_idxs[idx]: block_idxs[idx+1]]
        return block_datas

def get_guide_points(datas):
    keypoints = dict()
    for idx, block_datas in datas.items():
        values = []
        for data in block_datas:
            if "keypoint" in data:
                x_data = float(data.split(" ")[9].split("x:")[-1])
                y_data = float(data.split(" ")[10].split("y:")[-1])
                values.append([x_data, y_data])
        keypoints[idx] = values
    return keypoints


def get_catmull_points(datas):
    keypoints = dict()
    for idx, block_datas in datas.items():
        values = []
        for data in block_datas:
            if "after CatmullRom point" in data:
                x_data = float(data.split(" ")[11].split("x:")[-1])
                y_data = float(data.split(" ")[12].replace("]", "").split("y:")[-1])
                values.append([x_data, y_data])
        keypoints[idx] = values
    
    return keypoints


def get_fit_points(datas):
    keypoints = dict()
    for idx, block_datas in datas.items():
        values = []
        for data in block_datas:
            if "after Fit point" in data:
                x_data = float(data.split(" ")[11].split("x:")[-1])
                y_data = float(data.split(" ")[12].replace("]", "").split("y:")[-1])
                values.append([x_data, y_data])
        keypoints[idx] = values
    return keypoints


def plot_data(guide_points, catmull_points, fit_points):
    # Data for points (x, y coordinates)
    # Create a new figure and axis

    plt.figure()
    for key in guide_points:
        if key not in catmull_points or key not in fit_points:
            # print(key)
            continue
        # plt.figure()
        fig, (ax1, ax2, ax3) = plt.subplots(3)
        # Plot the points as a scatter plot
        frame_data = guide_points[key]
        x_points = [data[0] for data in frame_data]
        y_points = [data[1] for data in frame_data]
        # print("guide points x", x_points)
        ax1.scatter(x_points, y_points, c="blue")
        ax1.set_title('guide point')
        
        frame_data = catmull_points[key]
        x_points = [data[0] for data in frame_data]
        y_points = [data[1] for data in frame_data]
        # print("catmull_points x", x_points)
        ax2.scatter(x_points, y_points, c="red")
        ax2.set_title('catmull point')
        
        frame_data = fit_points[key]
        x_points = [data[0] for data in frame_data]
        y_points = [data[1] for data in frame_data]
        # print("fit_points x", x_points)
        ax3.scatter(x_points, y_points, c="green")
        ax3.set_title('fit point')
        
        # Set labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        # plt.title('Scatter Plot of Points')
        # Display the plot
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="show guide line")
    parser.add_argument("--f",  default="/media/wenhai/data/mapping/release/mal_x86/scripts/guide_line.txt", help="log file path")
    args = parser.parse_args()
    datas = extract_data_from_log_file(args.f)
    guide_points = get_guide_points(datas)
    # print(guide_points.keys())

    catmull_points = get_catmull_points(datas)
    # print(catmull_points.keys())
    
    
    fit_points = get_fit_points(datas)
    # print(fit_points.keys())


    plot_data(guide_points, catmull_points, fit_points)
    
    
    # a =dict()
    # b = list()
    # b.append([1,2])
    # b.append([2,3])
    # a[0] = b
    # b = []
    # print(a)

