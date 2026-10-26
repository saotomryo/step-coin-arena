# Docker詳細・トラブル対処

## 構成

ROS2 Jazzy / Ubuntu 24.04を基に、Genesis 1.4.3、CPU版PyTorch、PySide6、MuJoCo、Codex CLI、Xvfb、Openbox、VNC/noVNCを同じコンテナに配置します。Linuxのコンテナ環境を使うため、ホストにROS2を入れる必要はありません。

ベースイメージdigest・主要な直接依存・Codex CLI版を固定しています。過去のLinux ARM64の依存一覧は `docker/requirements-linux-arm64.snapshot.txt` です。これは完全な依存ロックや別CPUでの再現保証ではありません。

## 状態確認

```bash
docker compose ps
docker compose logs --tail 100 simulator
```

`healthy` は仮想デスクトップのnoVNCエンドポイントが応答したことを示し、競技やAI推論の成功までは意味しません。

## ブラウザが接続できない

Dockerを起動してからComposeを実行し、ログを確認してください。アプリの対戦設定ウィンドウを閉じるとコンテナも終了する構成です。再度 `docker compose up -d` で起動します。VNCクライアントなら `localhost:5900` でも接続できます。

既定のポートが使用中なら `.env.example` を `.env` にコピーして変更します。例えば `ROBOT_DESKTOP_PORT=6081` にした場合、アクセス先は `http://localhost:6081/vnc.html` です。VNC/noVNCは認証なしのため、loopback公開のまま使用してください。信頼できるローカルPCで利用します。

## 日本語入力・コピペ

noVNCのクリップボード連携はクライアントに依存します。直接入力が難しい場合はnoVNCのサイドバーのクリップボード機能やVNCクライアントを試してください。MacネイティブのIME操作と同じになることは保証していません。

## 遅い・画面が固まる

初回ビルドと初回物理初期化には時間がかかります。既定はCPUとソフトウェア描画です。高負荷なアプリを閉じ、装飾の少ない標準機体とルールベース戦略から試してください。競技時計はシミュレーション時間です。

## AIが動かない

コンテナ内で自分のCodexアカウントを認証し、利用可能なモデルと制限時間を確認してください。

```bash
docker compose run --rm simulator codex --version
docker compose run --rm simulator codex login status
```

Codex CLIのバージョンを変更する場合は `.env` に `CODEX_VERSION=...` を指定して再ビルドします。引数やモデルの互換性は別途確認が必要です。sandboxが使えない環境で、動作のためだけにprivileged指定へ変更することは想定していません。

生成・改善には、入力プロンプト、機体、制御、診断の抜粋をCodexへ送ります。機密情報を入力しないでください。認証情報やローカルログをGitへ追加する必要はありません。

## データ保存先を外部ディスクにする

既定はDocker管理の `resources` と `codex-auth` ボリュームです。Docker Desktopのディスクイメージも大きくなります。必要ならDocker Desktop側の保存先も設定してください。

`compose.override.yaml` をローカルに作り、`/data` を既存ディレクトリへ置き換えられます。

```yaml
services:
  simulator:
    volumes:
      - type: bind
        source: ${ROBOT_RESOURCE_DIR:?Set an existing resource directory}
        target: /data
        bind:
          create_host_path: false
```

`.env` に自分の `ROBOT_RESOURCE_DIR` を設定し、マウント済み・書込可能・コンテナのUID 10001からアクセス可能であることを確認してください。共有許可が必要なホストもあります。個人パスをリポジトリへコミットしないでください。

## コマンドだけ実行する

```bash
docker compose run --rm simulator python scripts/tune_robot.py --robot front_bogie_r20_wide --suite all --dry-run
```

## ローカル開発

標準配布はDockerです。ローカル開発ではPython仮想環境へ `requirements-genesis143.in` と `requirements-workshop.in` を入れ、ROS2環境を別途用意します。大きな依存があるため、初めて試す場合はDocker手順を使ってください。個人の既存仮想環境や保存機体はこのリポジトリに含めません。
