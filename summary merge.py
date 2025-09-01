import pandas as pd
import glob
import os

# CSV 파일들이 들어있는 폴더 경로
FOLDER_PATH = r"C:\123\logs\백업본"   # 경로 필요
OUTPUT_FILE = r"C:\123\logs\백업본\merged.csv" # 경로 필요

all_files = glob.glob(os.path.join(FOLDER_PATH, "*.csv"))

df_list = []

for file in all_files:
    # 파일 읽기
    df = pd.read_csv(file)
    
    # 파일명에서 abnormal 포함 여부 확인
    filename = os.path.basename(file).lower()
    if "abnormal" in filename:
        df["label"] = "abnormal"
    else:
        df["label"] = "normal"
    
    df_list.append(df)

# 하나로 합치기
merged_df = pd.concat(df_list, ignore_index=True)

# CSV 저장
merged_df.to_csv(OUTPUT_FILE, index=False, encoding="utf-8-sig")

print(f"저장 완료: {OUTPUT_FILE}")
