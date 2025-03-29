// WebRTC 설정
const configuration = {
    iceServers: [
        { urls: "stun:stun.l.google.com:19302" }
    ]
};
const peerConnection = new RTCPeerConnection(configuration);
const dataChannel = peerConnection.createDataChannel("joystickData");
const ws = new WebSocket("ws://localhost:8080");

dataChannel.onopen = () => console.log("Data channel opened");
dataChannel.onclose = () => console.log("Data channel closed");

ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "answer") {
        peerConnection.setRemoteDescription(new RTCSessionDescription(message.sdp));
    } else if (message.type === "candidate") {
        peerConnection.addIceCandidate(new RTCIceCandidate(message.candidate));
    }
};

peerConnection.onicecandidate = (event) => {
    if (event.candidate) {
        ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
    }
};

peerConnection.createOffer()
    .then(offer => peerConnection.setLocalDescription(offer))
    .then(() => {
        ws.send(JSON.stringify({ type: "offer", sdp: peerConnection.localDescription }));
    });

// ROS 2와 연결
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // rosbridge WebSocket 서버
});

ros.on('connection', () => {
    console.log('Connected to ROS 2 via rosbridge');
});

ros.on('error', (error) => {
    console.log('Error connecting to ROS 2:', error);
});

ros.on('close', () => {
    console.log('Disconnected from ROS 2');
});

// /joy 토픽 구독
const joyListener = new ROSLIB.Topic({
    ros: ros,
    name: '/joy',
    messageType: 'sensor_msgs/Joy'
});

joyListener.subscribe((message) => {
    console.log('Received joystick data:', message);
    const joystickData = {
        axes: message.axes,
        buttons: message.buttons
    };
    // HTML에 데이터 표시
    document.getElementById("joystick-data").innerHTML = `
        <p>X축: ${message.axes[0].toFixed(2)}</p>
        <p>Y축: ${message.axes[1].toFixed(2)}</p>
        <p>Z축: ${message.axes[2].toFixed(2)}</p>
        <p>버튼 잡기(LB): ${message.buttons[0]}</p>
        <p>버튼 잡기(RB): ${message.buttons[1]}</p>
    `;
    // WebRTC를 통해 PC2로 데이터 전송
    if (dataChannel.readyState === "open") {
        dataChannel.send(JSON.stringify(joystickData));
    }
});