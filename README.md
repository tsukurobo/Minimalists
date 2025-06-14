# Minimalists
2025年度キャチロボOBチームのリポジトリです

## 環境構築

1. vscode に `Raspberry Pi Pico Project` 拡張機能をインストールしてください
2. 新たに増えた`Raspberry Pi Pico Project`拡張機能メニューから`Project -> Configure CMake`を選択するとbuildディレクトリが生成されます

## 機能説明
### 動作フローチャート
![flowchart](img/FlowChart.drawio.png)
### マルチコア処理
Raspberry Pi Picoは2つのコアを持ち、マルチコア処理が可能です。
#### Core 0
RoboMaster M3508 P19 (ベース回転)，RoboMaster M2006 P36 (アーム並進)の制御を行います。
#### Core 1
シーケンス管理、センサーデータの取得、ハンドの制御を行います。

## 命名規則

本プロジェクトでは、Raspberry Pi Pico SDK（pico-sdk）の命名規則に準拠し、以下のように統一しています。

- **関数名・変数名・構造体名**  
    小文字スネークケース（例: `motor_speed`, `core1_entry`, `robot_state_t`）  
    構造体名など型名は末尾に `_t` を付けます。

- **マクロ・定数**  
    全大文字スネークケース（例: `MAX_SPEED`, `PICO_DEFAULT_LED_PIN`）

- **グローバル変数**  
    通常のスネークケースに接頭辞`g`を付けて衝突を避けます（例: `g_robot_state`）

- **ファイル名**  
    小文字スネークケース（例: `multicore.c`）

## コミットメッセージのタグ規則

コミットメッセージの先頭には、変更内容に応じて以下のタグを付けてください。

- `feat`: 新機能の追加
- `fix`: バグ修正
- `docs`: ドキュメントのみの変更
- `style`: フォーマットやスペースなど、動作に影響しない変更
- `refactor`: リファクタリング（機能追加やバグ修正を含まないコード変更）
- `test`: テストコードの追加・修正
- `chore`: ビルドプロセスや補助ツール、その他小さな変更

例:  
`feat: モーター制御機能を追加`  
`chore: コメントを修正`
