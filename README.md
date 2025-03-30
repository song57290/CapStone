3/30 17:26분 PC1와 연결된 조이스틱 값 http://127.0.0.1:3000/로 실시간 적용 성공

https://sincere-kindness-production.up.railway.app/
위 사이트는 시그널링 서버주소
https://railway.com/project/7e029104-22b4-4bdd-baa2-ecd271c88206?environmentId=00d64810-9316-4fc1-965a-8830ab73e8fb

3/30 19:58 PC1과 PC2 화면 연결 성공 데이터도 전송 o 대신 임의의 숫자가 넘어가는 중임

3/30 21:01 PC1과 PC2 화면 연결 성공 (재생버튼을 누르면 동작)




각 클라이언트 폴더에서 아래 명령어를 사용하세요.

PC1 클라이언트 실행 (예: 포트 3000):

bash
복사
cd ~/CapStone/webrtc-robot-arm/client/pc1
npx http-server -p 3000
브라우저에서 http://127.0.0.1:3000로 접속하세요.

PC2 클라이언트 실행 (예: 포트 3001):

bash
복사
cd ~/CapStone/webrtc-robot-arm/client/pc2
npx http-server -p 3001
브라우저에서 http://127.0.0.1:3001 (또는 PC2의 실제 IP와 포트)로 접속하세요.