// joystick_server.js (예시)
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', ws => {
  console.log('클라이언트 연결됨');

  ws.on('message', message => {
    console.log('수신 데이터:', message);
    wss.clients.forEach(client => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  });
});

console.log('WebSocket 서버 실행중: ws://127.0.0.1:8080');
