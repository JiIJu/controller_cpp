#!/bin/bash

WATCHED_DIR="/home/flyingcar/controller_cpp/fire_detection"  # 모니터링할 디렉토리 경로
GIT_REPO="/home/flyingcar/controller_cpp"  # Git 저장소 경로

# inotifywait을 사용하여 파일 생성 및 수정 이벤트를 모니터링
inotifywait -m -e create -e modify -e close_write "$WATCHED_DIR" | while read -r directory events filename; do
    echo "Detected file change: $filename in $directory"
    
    # Git 명령 실행
    cd "$GIT_REPO"

    # 변경 사항이 있는지 확인
    if git status --porcelain | grep "$filename"; then
        # 변경된 파일만 추가
        git add "$WATCHED_DIR/$filename"  # 변경된 파일 추가
        git commit -m "Auto-commit: Added or modified $filename"  # 자동 커밋 메시지
        
        # Git push 실행 후 성공 여부 확인
        if git push origin main; then
            echo "File $filename has been pushed to remote repository"
        else
            echo "Error: Failed to push $filename to remote repository"
        fi
    else
        echo "No changes to commit for $filename"
    fi

    # 짧은 대기 시간 추가 (필요 시 조정 가능)
    sleep 3
done