# ArduinoBaseTest
これはベース部分の**テスト用**のArduinoスケッチです。
ロボマスのベース部分の制御を行います。
**本番用じゃないです。**

## ArduinoBaseTest.ino
ArduinoBaseTest.inoは、ロボマスのベース部分の制御を行うためのスケッチです。このスケッチでは、ロボマスのモーターを制御し、動的な挙動をシミュレートします。

## Logger.py
Logger.pyは、ロボマスの動作ログを記録するためのPythonスクリプトです。このスクリプトは、Arduinoからのシリアルデータを受信し、ログファイルに保存します。

### Logger.pyの使い方
1. Python環境をセットアップします。(venvなどを使用して仮想環境を作成することを推奨します。)
   ```bash
   python -m venv venv
   source venv/bin/activate  # Windowsの場合は venv\Scripts\activate
   ```
2. 必要なライブラリをインストールします。
   ```bash
   pip install -r requirements.txt
   ```
   または個別にインストールする場合：
   ```bash
   pip install pyserial scipy numpy matplotlib
   ```
3. ArduinoをPCに接続し、ArduinoBaseTest.inoをアップロードします。
4. スクリプト内の`SERIAL_PORT`変数を、使用しているArduinoのシリアルポートに変更します。
   ```python
   SERIAL_PORT = 'COM4'  # Windowsの場合
   # LinuxやMacの場合は '/dev/ttyUSB0' など
   ```
5. Logger.pyを実行します。
   ```bash
   python Logger.py
   ```
6. ログが記録され、`log`ディレクトリに保存されます。

### Logger.pyの設定
Logger.pyの`START_COMMAND`で動作がちょっといじれます
```python
START_COMMAND = "c"
```
`"c"`だと、ArduinoBaseTest.inoにある一連の動作が実行されます。
`"a1"`だと、電流指令値1Aでモーターを回転させます。aの後の数字が電流値です。

## ParamIdentify.py
ParamIdentify.pyは、ロボマスの動的パラメータを同定するためのPythonスクリプトです。このスクリプトは、Logger.pyで記録されたログデータを解析し、ロボマスの動的特性を推定します。

### ParamIdentify.pyの使い方
1. Logger.pyで記録されたログデータを用意します。
2. ParamIdentify.pyを実行します。
   ```bash
   python ParamIdentify.py
   ```
3. スクリプトはログデータを解析し、ロボマスの動的パラメータを推定します。
4. 結果はコンソールに表示されます。
