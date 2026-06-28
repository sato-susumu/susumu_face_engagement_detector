# outputs/

すべての評価実行・可視化の成果物がここに集約されます。

```
outputs/
├── baselines/        # 評価結果 JSON (run_*_eval.py の出力)
│                     # 命名規約: v<n>_<backend>_<dataset>[_smoke|_full].json
├── reports/          # Markdown レポート (generate_report.py の出力)
│                     # REPORT.md は最新のすべての baselines を統合
├── figures/          # 可視化 PNG (visualize.py の出力)
│   ├── detection/    # 検出オーバーレイ画像、AP/latency 棒グラフ
│   ├── recognition/  # ROC曲線、コサイン類似度分布
│   └── engagement/   # スコア時系列、状態遷移
└── runs/             # 個別実行ログ・rosbag・スクラッチ出力
    └── video_demo/   # 動画入力と注釈付き出力 MP4 (`make video-demo`)
```

## 主要コマンド

```bash
make eval-detection-baseline   # outputs/baselines/v0_dlib_hog_wider_val.json
make eval-detection-yunet      # outputs/baselines/v1_yunet_wider_val.json
make eval-recognition-baseline # outputs/baselines/v0_dlib_128d_lfw.json
make visualize                 # outputs/figures/ 配下に PNG 一式
make video-demo                # public domain 動画を入力に、注釈付き MP4 を生成 (OpenVINO gaze model があれば視線も描画)
make video-demo-multi          # 複数人物がはっきり映る長尺 public domain 動画で MP4 を生成
make chokepoint-gt-demo        # 正解人物ID付き ChokePoint 顔IDデモ MP4 + JSON を生成
make eval-report               # outputs/reports/REPORT.md (図リンク埋め込み)
make outputs                   # 上記すべてを順番に実行
```

## 再現性メモ

- `baselines/*.json` は決定的 — 同じデータ・同じ重み・同じパラメータなら数値も同じ
- `figures/*.png` も同様 (matplotlib のスタイル設定固定)
- `runs/` は実行ごとに上書き・追記される一時領域

## Git方針

このディレクトリの中身は **基本的に git 管理外** です (`.gitignore` で除外)。
公開したい結果は別途 `docs/` に手動でコピーしてください。
