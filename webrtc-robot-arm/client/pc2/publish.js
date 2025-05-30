// publish.js
// ROS 연결
const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => console.log('ROS 연결 성공'));
ros.on('error', (error) => console.error('ROS 연결 오류:', error));
ros.on('close', () => console.log('ROS 연결 종료'));

// Joy Publisher 생성
const joyPublisher = new ROSLIB.Topic({
  ros: ros,
  name: '/joy',    // Gazebo에서 구독하는 토픽 이름
  messageType: 'sensor_msgs/Joy'
});

// WebRTC 연결 준비
const ws = new WebSocket("ws://192.168.35.2:8080");
const configuration = {
  iceServers: [
    { urls: "stun:stun.l.google.com:19302" },
    { urls: "turn:192.168.35.2:3478", username: "user", credential: "pass" }
  ]
};

const pc = new RTCPeerConnection(configuration);

// ICE 후보 전송
pc.onicecandidate = (event) => {
  if (event.candidate) {
    ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
  }
};

// 데이터 채널 수신
pc.ondatachannel = (event) => {
  const dataChannel = event.channel;
  dataChannel.onopen = () => console.log("데이터 채널 열림");
  dataChannel.onclose = () => console.log("데이터 채널 닫힘");
  dataChannel.onmessage = (event) => {
    try {
      const joystickData = JSON.parse(event.data);
      console.log("수신된 조이스틱 데이터:", joystickData);

      // ROS Joy 메시지 생성 및 퍼블리시
      const joyMsg = new ROSLIB.Message({
        axes: joystickData.axes,
        buttons: joystickData.buttons
      });

      joyPublisher.publish(joyMsg);
      console.log("ROS에 Joy 메시지 전송:", joyMsg);

    } catch (e) {
      console.error("데이터 처리 오류:", e);
    }
  };
};

// 시그널링 메시지 처리
ws.onmessage = async (event) => {
  const message = JSON.parse(event.data);
  if (message.type === "offer") {
    await pc.setRemoteDescription(new RTCSessionDescription(message));
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    ws.send(JSON.stringify(answer));
  } else if (message.type === "candidate") {
    await pc.addIceCandidate(new RTCIceCandidate(message.candidate));
  }
};

ws.onopen = () => console.log("시그널링 서버 연결됨");
ws.onerror = (error) => console.error("시그널링 오류:", error);

console.log("publish.js 실행 완료!");