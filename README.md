# Robot@Factory

## 概要
`Robot@Factory`は、Isaac Simを使用した工場環境でのロボットシミュレーションをサポートするプロジェクトです。<br>
このリポジトリには、Melonロボットを制御するActorおよびBehavior Treeが含まれています。

---

## 使用方法

### 1. リポジトリのクローン
以下のコマンドを実行してリポジトリをクローンします。

```shell
git clone http://192.168.128.23/satoya_sugimoto/factory_app.git
cd factory_app
```

### 2. Dockerの起動
Dockerコンテナの名前やROS_DOMAIN_IDは`.env`ファイルでカスタムできます。

```shell
# .envファイルの例
ROS_DOMAIN_ID=80                      # ROSドメインID（デフォルト: 80）

CONTAINER_NAME=factory_app            # コンテナ名（デフォルト: factory_app)
```

変更できたら次のコマンドを実行してDockerコンテナを起動します。

```bash
build/pc.sh
```

これで、`.env`ファイルで指定した名前のDockerが起動します。Xウィンドウは表示されません。  

## 推奨環境
VSCodeを使用して、Remote ExplorerからDockerにアタッチすることを推奨します。

1. VSCodeのターミナルを開き、以下のコマンドを実行します。

```shell
cd pytwb_ws
pytwb
> create cm1
> Y
```
を実行します。  

2. 上記の手順を完了後、以下のコマンドでActorおよびBehavior Treeを使用できます。

```shell
actor
```

---

## 注意事項
- Dockerコンテナは自動で停止しないため、使用後は手動で停止してください。
- 2回目以降は、Dockerが起動していなくても、VSCode内で直接`docker attach`を実行することで、VSCodeがDockerを起動します。

