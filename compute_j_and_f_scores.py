import os
import numpy as np
import cv2
from sklearn.metrics import f1_score
import pandas as pd
# Load grayscale image for prediction
def load_prediction_image(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    return img

# Load and extract red channel for ground truth
def load_ground_truth_image(image_path):
    img = cv2.imread(image_path)
    # Extract red channel
    red_channel = img[:, :, 2]
    # Threshold the red channel to create a binary mask
    _, red_bin = cv2.threshold(red_channel, 127, 1, cv2.THRESH_BINARY)
    return red_bin

# Calculate Jaccard Index (IoU)
def j_score(ground_truth, prediction):
    intersection = np.logical_and(ground_truth, prediction)
    union = np.logical_or(ground_truth, prediction)
    iou_score = np.sum(intersection) / np.sum(union)
    return iou_score

# Calculate F-score
def f_score(ground_truth, prediction):
    ground_truth_flat = ground_truth.flatten()
    prediction_flat = prediction.flatten()
    # Using F1 score from sklearn to calculate precision-recall based F-score
    f1 = f1_score(ground_truth_flat, prediction_flat)
    return f1

# Function to evaluate segmentation for a single pair of images
def evaluate_single_pair(ground_truth_path, prediction_path):
    ground_truth = load_ground_truth_image(ground_truth_path)
    prediction = load_prediction_image(prediction_path)
    
    # Binarize the predicted mask
    _, prediction_bin = cv2.threshold(prediction, 127, 1, cv2.THRESH_BINARY)
    
    # Calculate the scores
    jaccard = j_score(ground_truth, prediction_bin)
    f1 = f_score(ground_truth, prediction_bin)
    
    return jaccard, f1

# Main function to evaluate all images in the directories
def evaluate_all_images(ground_truth_dir, prediction_dir):
    ground_truth_files = sorted(os.listdir(ground_truth_dir))
    prediction_files = sorted(os.listdir(prediction_dir))
    
    total_j_score = 0
    total_f_score = 0
    count = 0
    
    for gt_file, pred_file in zip(ground_truth_files, prediction_files):
        gt_path = os.path.join(ground_truth_dir, gt_file)
        pred_path = os.path.join(prediction_dir, pred_file)
        
        if os.path.isfile(gt_path) and os.path.isfile(pred_path):
            jaccard, f1 = evaluate_single_pair(gt_path, pred_path)
            total_j_score += jaccard
            total_f_score += f1
            count += 1
            print(f"Processed {gt_file} - J Score: {jaccard}, F Score: {f1}")
    
    # Compute the average scores
    avg_j_score = total_j_score / count if count > 0 else 0
    avg_f_score = total_f_score / count if count > 0 else 0
    
    return avg_j_score, avg_f_score

def evaluate_dataset(dataset_path,year):
    results = []
    base_pred_path = f"{dataset_path}/results"
    # 予測結果のフォルダを取得
    pred_folders = [f for f in os.listdir(base_pred_path) if os.path.isdir(os.path.join(base_pred_path, f))]
    print(pred_folders)
    for task in pred_folders:
        ground_truth_folder = f"{dataset_path}/Annotations/480p/{task}"
        prediction_folder = f"{dataset_path}/results/{task}/mask_target1"
        
        if not os.path.exists(ground_truth_folder):
            print(f"Warning: Ground truth folder not found for task {task}")
            continue
        
        avg_jaccard, avg_f1 = evaluate_all_images(ground_truth_folder, prediction_folder)
        
        results.append({
            'Year': year,
            'Task': task,
            'Average Jaccard': avg_jaccard,
            'Average F1': avg_f1
        })
    
    return results

def main_2016():
    all_results = []
    data_root = "/media/medical/Data/yamada/davis_evaluation"
    for year in [2016]:
        dataset_path = f"{data_root}/DAVIS_{year}"
        year_results = evaluate_dataset(dataset_path,year)
        all_results.extend(year_results)
    df = pd.DataFrame(all_results)
    df.to_csv(f'{data_root}/DAVIS_{year}/davis_evaluation_results.csv', index=False)

    # 年ごとの平均と分散を計算
    summary_stats = []
    for year in [2016]:
        year_df = df[df['Year'] == year]
        summary_stats.append({
            'Year': year,
            'Avg Jaccard Mean': year_df['Average Jaccard'].mean(),
            'Avg Jaccard Var': year_df['Average Jaccard'].var(),
            'Avg F1 Mean': year_df['Average F1'].mean(),
            'Avg F1 Var': year_df['Average F1'].var()
        })

    # # 全体の平均と分散を計算
    # summary_stats.append({
    #     'Year': 'All',
    #     'Avg Jaccard Mean': df['Average Jaccard'].mean(),
    #     'Avg Jaccard Var': df['Average Jaccard'].var(),
    #     'Avg F1 Mean': df['Average F1'].mean(),
    #     'Avg F1 Var': df['Average F1'].var()
    # })

    # 要約統計をDataFrameに変換
    summary_df = pd.DataFrame(summary_stats)

    # 要約統計を表示
    print("\nSummary Statistics:")
    print(summary_df)

    # 要約統計をCSVファイルに保存
    summary_df.to_csv(f'{data_root}/DAVIS_{year}/davis_evaluation_summary.csv', index=False)
    print("Summary statistics have been saved to 'davis_evaluation_summary.csv'")

if __name__ == "__main__":
    main_2016()
    # # Example usage
    # ground_truth_folder = '/home/safetyu-desktop1/benchmarks_related/davis-2017/DAVIS/Annotations/480p/bear'
    # prediction_folder = '/home/safetyu-desktop1/ist_ros/benchmark/mask_target1'
    # for task in task_list:
    #     avg_jaccard, avg_f1 = evaluate_all_images(ground_truth_folder, prediction_folder)
    # print(f"Average J Score (IoU): {avg_jaccard}")
    # print(f"Average F Score: {avg_f1}")


    # import matplotlib.pyplot as plt

    # # Load and display an example pair
    # ground_truth_image = load_ground_truth_image(f"{ground_truth_folder}/00000.png")
    # prediction_image = load_prediction_image(f"{prediction_folder}/mask_target1_000001.png")

    # # plt.figure(figsize=(10, 5))
    # # plt.subplot(1, 2, 1)
    # # plt.imshow(ground_truth_image, cmap='gray')
    # # plt.title('Ground Truth Mask')

    # # plt.subplot(1, 2, 2)
    # # plt.imshow(prediction_image, cmap='gray')
    # # plt.title('Prediction Mask')

    # # plt.show()


    # _, ground_truth_bin = cv2.threshold(ground_truth_image, 127, 1, cv2.THRESH_BINARY)
    # _, prediction_bin = cv2.threshold(prediction_image, 127, 1, cv2.THRESH_BINARY)

    # plt.figure(figsize=(10, 5))
    # plt.subplot(1, 2, 1)
    # plt.imshow(ground_truth_bin, cmap='gray')
    # plt.title('Binarized Ground Truth Mask')

    # plt.subplot(1, 2, 2)
    # plt.imshow(prediction_bin, cmap='gray')
    # plt.title('Binarized Prediction Mask')

    # plt.show()