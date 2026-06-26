#!/usr/bin/env python3
"""
著作権フリーのテスト用顔画像生成スクリプト
純粋に合成された画像なので著作権の問題なし
"""

import cv2
import numpy as np
import os

def generate_test_face(width=640, height=480, face_id="test_face", **kwargs):
    """合成顔画像を生成"""
    
    # 背景色の設定（顔IDに基づいて変更）
    bg_colors = {
        "person1": [120, 150, 180],
        "person2": [180, 120, 150], 
        "person3": [150, 180, 120],
        "test_face": [100, 120, 140]
    }
    bg_color = bg_colors.get(face_id, [100, 120, 140])
    
    # 背景作成
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)
    
    # 顔の基本設定
    face_center_x = kwargs.get('face_x', width // 2)
    face_center_y = kwargs.get('face_y', height // 2)
    face_size = kwargs.get('face_size', 120)
    
    # 顔の肌色設定（IDに基づいて変更）
    skin_colors = {
        "person1": (220, 180, 160),
        "person2": (200, 170, 150),
        "person3": (240, 200, 180),
        "test_face": (210, 175, 155)
    }
    skin_color = skin_colors.get(face_id, (210, 175, 155))
    
    # 顔の輪郭
    face_axes = (face_size // 2, int(face_size * 0.7))
    cv2.ellipse(img, (face_center_x, face_center_y), face_axes, 0, 0, 360, skin_color, -1)
    
    # 目の設定
    eye_y = face_center_y - face_size // 4
    eye_distance = face_size // 3
    left_eye_x = face_center_x - eye_distance // 2
    right_eye_x = face_center_x + eye_distance // 2
    
    eye_size = kwargs.get('eye_size', 12)
    
    # 左目
    cv2.circle(img, (left_eye_x, eye_y), eye_size, (255, 255, 255), -1)
    cv2.circle(img, (left_eye_x + 2, eye_y), eye_size // 2, (50, 50, 50), -1)
    cv2.circle(img, (left_eye_x + 3, eye_y - 1), 2, (255, 255, 255), -1)  # ハイライト
    
    # 右目
    cv2.circle(img, (right_eye_x, eye_y), eye_size, (255, 255, 255), -1)
    cv2.circle(img, (right_eye_x + 2, eye_y), eye_size // 2, (50, 50, 50), -1)
    cv2.circle(img, (right_eye_x + 3, eye_y - 1), 2, (255, 255, 255), -1)  # ハイライト
    
    # 眉毛
    eyebrow_y = eye_y - eye_size - 5
    cv2.ellipse(img, (left_eye_x, eyebrow_y), (15, 4), 0, 0, 180, (100, 80, 60), -1)
    cv2.ellipse(img, (right_eye_x, eyebrow_y), (15, 4), 0, 0, 180, (100, 80, 60), -1)
    
    # 鼻
    nose_x = face_center_x
    nose_y = face_center_y - 5
    nose_points = np.array([
        [nose_x, nose_y - 10],
        [nose_x - 6, nose_y + 8],
        [nose_x + 6, nose_y + 8]
    ], np.int32)
    
    # より濃い肌色で鼻を描画
    nose_color = tuple(max(0, c - 20) for c in skin_color)
    cv2.fillPoly(img, [nose_points], nose_color)
    
    # 鼻孔
    cv2.circle(img, (nose_x - 3, nose_y + 5), 1, (80, 60, 40), -1)
    cv2.circle(img, (nose_x + 3, nose_y + 5), 1, (80, 60, 40), -1)
    
    # 口
    mouth_x = face_center_x
    mouth_y = face_center_y + face_size // 3
    mouth_width = kwargs.get('mouth_width', 30)
    
    # 口の形状（IDに基づいて変更）
    mouth_expressions = {
        "person1": ("smile", 8),
        "person2": ("neutral", 5),
        "person3": ("slight_smile", 6),
        "test_face": ("smile", 7)
    }
    expression, mouth_height = mouth_expressions.get(face_id, ("smile", 7))
    
    if expression == "smile":
        cv2.ellipse(img, (mouth_x, mouth_y), (mouth_width, mouth_height), 
                   0, 0, 180, (150, 80, 80), -1)
        # 歯
        cv2.ellipse(img, (mouth_x, mouth_y - 2), (mouth_width - 6, 3), 
                   0, 0, 180, (255, 255, 255), -1)
    else:
        cv2.line(img, (mouth_x - mouth_width // 2, mouth_y), 
                (mouth_x + mouth_width // 2, mouth_y), (120, 80, 80), 3)
    
    # 髪の毛（簡単な表現）
    hair_color = (60, 40, 20)  # 茶色
    hair_top = face_center_y - int(face_size * 0.6)
    hair_width = int(face_size * 0.8)
    
    # 髪の毛の楕円
    cv2.ellipse(img, (face_center_x, hair_top), (hair_width // 2, face_size // 3), 
               0, 0, 180, hair_color, -1)
    
    # 情報テキスト追加
    cv2.putText(img, f'ID: {face_id}', (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, f'Synthetic Face', (10, 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, f'Size: {width}x{height}', (10, height - 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return img

def generate_multiple_faces_image(width=640, height=480):
    """複数の顔が含まれる画像を生成"""
    img = np.full((height, width, 3), [80, 100, 120], dtype=np.uint8)
    
    # 3つの顔を配置
    face_positions = [
        {"face_x": width // 4, "face_y": height // 2, "face_size": 80},
        {"face_x": width // 2, "face_y": height // 3, "face_size": 90},
        {"face_x": 3 * width // 4, "face_y": 2 * height // 3, "face_size": 85}
    ]
    
    face_ids = ["person1", "person2", "person3"]
    
    for i, (face_id, pos) in enumerate(zip(face_ids, face_positions)):
        # 各顔を個別に生成して合成
        face_img = generate_test_face(width, height, face_id, **pos)
        
        # 顔部分のマスク作成
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.ellipse(mask, (pos["face_x"], pos["face_y"]), 
                   (pos["face_size"] // 2, int(pos["face_size"] * 0.7)), 
                   0, 0, 360, 255, -1)
        
        # マスクを使って顔を合成
        img = np.where(mask[..., None] > 0, face_img, img)
    
    # タイトル追加
    cv2.putText(img, 'Multiple Faces Test Image', (width // 2 - 120, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return img

def main():
    """テスト画像を生成"""
    # 出力ディレクトリ
    output_dir = "test_images"
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating copyright-free synthetic test images...")
    
    # 単体顔画像生成
    face_configs = [
        {"face_id": "person1", "face_x": 320, "face_y": 240, "face_size": 120},
        {"face_id": "person2", "face_x": 300, "face_y": 250, "face_size": 100},
        {"face_id": "person3", "face_x": 350, "face_y": 230, "face_size": 110}
    ]
    
    for config in face_configs:
        img = generate_test_face(640, 480, **config)
        filename = f"{output_dir}/{config['face_id']}_test.jpg"
        cv2.imwrite(filename, img)
        print(f"Generated: {filename}")
    
    # 複数顔画像
    multi_face_img = generate_multiple_faces_image(640, 480)
    multi_filename = f"{output_dir}/multiple_faces_test.jpg"
    cv2.imwrite(multi_filename, multi_face_img)
    print(f"Generated: {multi_filename}")
    
    # 異なるサイズの画像
    sizes = [(320, 240), (800, 600), (1280, 720)]
    for w, h in sizes:
        img = generate_test_face(w, h, "test_face", face_x=w//2, face_y=h//2, face_size=min(w, h)//6)
        filename = f"{output_dir}/test_face_{w}x{h}.jpg"
        cv2.imwrite(filename, img)
        print(f"Generated: {filename}")
    
    # 顔なし画像（背景のみ）
    no_face_img = np.full((480, 640, 3), [100, 120, 140], dtype=np.uint8)
    cv2.putText(no_face_img, 'No Face Test Image', (200, 240), 
               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imwrite(f"{output_dir}/no_face_test.jpg", no_face_img)
    print(f"Generated: {output_dir}/no_face_test.jpg")
    
    print(f"\nAll test images generated in '{output_dir}' directory")
    print("These are purely synthetic images with no copyright issues.")

if __name__ == '__main__':
    main()