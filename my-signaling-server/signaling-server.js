// signaling-server.js
const express = require('express');
const http = require('http');
const { Server } = require('socket.io');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

io.on('connection', socket => {
  console.log(`클라이언트 연결됨: ${socket.id}`);

  socket.on('signal', data => {
    // 같은 룸에 있는 다른 클라이언트에게 신호 데이터 전달
    socket.broadcast.emit('signal', data);
  });
});

const PORT = process.env.PORT || 3002;
server.listen(PORT, () => console.log(`시그널링 서버 실행 중: 포트 ${PORT}`));
