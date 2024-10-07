#!/bin/bash

WATCHED_DIR="/home/flyingcar/controller_cpp/fire_detection"  # 모니터링할 디렉토리 경로
GIT_REPO="/home/flyingcar/controller_cpp"  # Git 저장소 경로

# inotifywait을 사용하여 파일 생성 이벤트를 모니터링
inotifywait -m -e create "$WATCHED_DIR" | while read -r directory events filename; do
    echo "Detected new file: $filename in $directory"
    
    # Git 명령 실행
    cd "$GIT_REPO"
    
    git add .  # 새 파일 추가
    git commit -m "Auto-commit: Added"  # 자동 커밋 메시지
    git push origin main  # 변경 사항을 원격 저장소로 push
    
    echo "File $filename has been pushed to remote repository"
done
