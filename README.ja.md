# TM5-700 自然言語・視覚ベース色分けシステム

[English](README.md) | 日本語

このリポジトリは、ROS 2 Humble と Gazebo を使った TM5-700 ロボットアームの
シミュレーションプロジェクトです。自然言語コマンドを解釈し、ツール先端の
カメラで色付きブロックを認識し、ピックアンドプレース動作を計画・実行し、
最後に視覚情報で結果を確認します。

現在の目的は、固定スクリプトで 1 個のブロックを動かすことだけではありません。
次の 4 つの層を 1 つの実験ワークフローとして接続しています。

1. TM5-700、作業台、吸着ツール、色付きブロックの Gazebo シーン。
2. カメラ画像による赤・黄・青ブロックの 3x3 グリッド位置検出。
3. 自然言語コマンドの解析とタスク計画。
4. MoveIt/Gazebo による実行、吸着・解放、実行後の視覚確認。

## デモ資料

| 資料 | 内容 |
| --- | --- |
| [中期報告 PDF](docs/tm5-700_midterm_report.pdf) | システム設計、進捗、実験方針をまとめた発表資料です。 |
| [demo_5.gif](media/demo_5.gif) | Demo 5 の視覚閉ループ実験をすぐ確認できるプレビューです。 |
| [demo_5.mp4](media/demo_5.mp4) | 視覚フィードバック、自然言語計画、実行、結果確認を行う閉ループ実験です。 |
| [demo_6.gif](media/demo_6.gif) | Demo 6 の色分け実験をすぐ確認できるプレビューです。 |
| [demo_6.mp4](media/demo_6.mp4) | LLM によるコマンド分類、固定色ターゲット、実行、最終確認を行う色分け実験です。 |

## このプロジェクトで行うこと

シーンには赤・黄・青のブロックが 3x3 グリッド上に配置されています。ユーザーは
色と位置を指定してロボットに指示できます。システムはその指示をグリッド上の
操作に変換し、TM5-700 のシミュレーションで実行し、観測されたシーンと論理状態
ファイルを同期します。

主な構成要素は次の通りです。

- 通常の色付きブロックシーンと Demo 6 用色分けテーブルの Gazebo world。
- ツール先端カメラと吸着カップを持つ TM5-700 モデル。
- MoveIt によるシミュレーション制御用 launch ファイル。
- `/tool_camera/image_raw` 用のカメラ bridge と画像ビューア設定。
- グリッド、ブロック、ピック・プレース姿勢の設定ファイル。
- Gazebo detachable joint を使った吸着・解放サービス。
- 固定動作から LLM 支援の視覚的色分けまでのデモプログラム。

## システムの流れ

```text
自然言語コマンド
        |
        v
LLM / ルールベース解析
        |
        v
グリッド上のタスク計画
        |
        v
MoveIt による関節姿勢実行
        |
        v
Gazebo の吸着・解放
        |
        v
カメラによる結果確認
```

## デモの流れ

| Demo | 目的 | 主な機能 |
| --- | --- | --- |
| `demo_1` | 固定された赤ブロック移動 | ready -> pick -> attach -> place -> detach の基本動作を確認します。 |
| `demo_2` | ルールベース言語コマンド | 簡単な中国語・英語の色とグリッド位置を解析します。 |
| `demo_3` | LLM による順序付き移動 | LLM で単一ブロックの順序付きコマンドを解析します。 |
| `demo_4` | LLM + プランナー | occupied target、swap、region、複数ステップの計画を扱います。 |
| `demo_5` | 視覚閉ループ | 実行前にカメラで状態を検出し、実行後に結果を確認します。 |
| `demo_6` | 視覚的色分け | 専用テーブル上で色ごとの固定ターゲットへ分類します。 |

## Demo 5: 視覚閉ループ

![Demo 5 視覚閉ループプレビュー](media/demo_5.gif)

`demo_5` は `demo_4` に視覚フィードバックを追加したものです。計画前にアームを
`ready` 姿勢へ移動し、`/tool_camera/image_raw` を読み取り、ブロック位置を検出し、
カメラ上のセルを論理グリッドへ変換して `cube_state.yaml` に保存します。その後、
`demo_4` と同じ言語解析、計画、実行を行います。実行後は再び `ready` に戻り、
カメラで要求されたブロックが目標セルに到達したか確認します。

視覚検出のみを確認する場合:

```bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p vision_only:=true
```

視覚検出と計画だけを確認し、実際のピックアンドプレースを行わない場合:

```bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p plan_only:=true
```

完全な視覚閉ループ実験:

```bash
ros2 run tm5_color_pick_demo demo_5
```

Demo 5 はデフォルトで実行前確認を行います。`camera_state`、`current_state`、
`planned_stages` を確認してから `y` を入力します。

## Demo 6: LLM 色分け

![Demo 6 色分けプレビュー](media/demo_6.gif)

`demo_6` は、3x3 の色分けテーブルを持つ専用 Gazebo world を使います。ユーザーは
自然言語で色分けを指示します。LLM はそのコマンドが対応可能な色分けタスクかを
分類しますが、目標セルはプログラム側で固定しているため、結果は決定的です。

```yaml
yellow: right_top
blue: right_middle
red: right_bottom
```

Demo 6 用 Gazebo + MoveIt world を起動:

```bash
ros2 launch tm5_color_pick_demo color_cubes_demo_6_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

視覚検出のみ:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p vision_only:=true
```

計画のみ:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p plan_only:=true
```

完全な色分け実験:

```bash
ros2 run tm5_color_pick_demo demo_6
```

`current_state`、`target_state`、`action_plan` を表示した後、Demo 6 は実行確認を
求めます。実行後にもう一度カメラで確認し、黄・青・赤が固定ターゲットに到達
したかを検証します。

## リポジトリ構成

```text
docs/
  tm5-700_midterm_report.pdf
media/
  demo_5.mp4
  demo_6.mp4
src/tm5_color_pick_demo/
  config/
  launch/
  tm5_color_pick_demo/
  worlds/
  xacro/
```

重要なファイル:

| Path | 役割 |
| --- | --- |
| `src/tm5_color_pick_demo/config/grid_layout.yaml` | 3x3 論理グリッド定義。 |
| `src/tm5_color_pick_demo/config/cube_state.yaml` | 実行時のブロック論理状態。 |
| `src/tm5_color_pick_demo/config/grid_pose_groups.yaml` | 記録済みのピック・プレース関節姿勢。 |
| `src/tm5_color_pick_demo/worlds/color_cubes.sdf` | 通常の赤・黄・青ブロック world。 |
| `src/tm5_color_pick_demo/launch/color_cubes_moveit_gazebo.launch.py` | 標準の Gazebo + MoveIt launch ファイル。 |
| `src/tm5_color_pick_demo/launch/color_cubes_demo_6_moveit_gazebo.launch.py` | Demo 6 色分け world 用 launch ファイル。 |
| `src/tm5_color_pick_demo/tm5_color_pick_demo/suction_grasp_manager.py` | Gazebo detachable-joint 吸着サービス。 |

## ビルド

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
cd ~/Desktop/tm5_color_pick_demo
colcon build --packages-select tm5_color_pick_demo
source install/setup.bash
```

デスクトップ名がローカライズされている場合は、`~/Desktop/tm5_color_pick_demo` を
実際のチェックアウト先に置き換えてください。

## 標準実行手順

Terminal A, Gazebo + MoveIt:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

Terminal B, 任意のカメラビューア:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run rqt_image_view rqt_image_view /tool_camera/image_raw
```

Terminal C, 吸着サービス:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

吸着サービスの確認:

```bash
ros2 service list | grep suction
```

期待されるサービス:

```text
/suction/attach_red
/suction/detach_red
/suction/attach_yellow
/suction/detach_yellow
/suction/attach_blue
/suction/detach_blue
```

## 論理状態のリセット

各デモは `cube_state.yaml` を現在の論理状態として使用します。Gazebo を再起動した
後や、途中で失敗してファイルとシーンが一致しなくなった場合はリセットします。

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo reset_cube_state
```

デフォルト状態:

```yaml
red: left_top
yellow: left_middle
blue: left_bottom
```

## 現在の制限

- 視覚検出は、固定された `ready` 視点から赤・黄・青ブロックが明確に見えることを
  前提にしています。
- ブロックは Gazebo 上で制御され、論理状態として追跡されますが、MoveIt の
  planning scene collision object として完全には同期していません。
- 吸着ツールは物理的な真空グリッパではなく、Gazebo detachable joint による
  シミュレーション機構です。
- Demo 6 の色ターゲットは固定です。LLM はタスク分類を行いますが、ターゲット位置
  自体は選びません。

## 開発メモ

このプロジェクトでは、3x3 グリッドのピック・プレースに対して、実行時に新しい
Cartesian IK ターゲットを生成するのではなく、記録済みの関節姿勢を使用します。
これにより、現在の作業セルでは経路の再現性が上がり、TM5-700 の IK の曖昧さを
減らせます。
