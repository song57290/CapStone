// 직장 환경: PC1 (172.22.156.216), PC2 (172.17.3.215)
//const ws = new WebSocket("ws://172.22.156.216:8080");

// 집 환경: PC1 (192.168.35.2), PC2 (192.168.35.25)
const ws = new WebSocket("ws://192.168.35.2:8080");

const configuration = {
    iceServers: [
        { urls: "stun:stun.l.google.com:19302" },
        // TURN 서버 설정 (집에서도 직장 환경과 동일하게 사용)
        // { urls: "turn:your-turn-server", username: "user", credential: "pass" }
    ]
};

const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
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

const joyListener = new ROSLIB.Topic({
    ros: ros,
    name: '/joy',
    messageType: 'sensor_msgs/Joy'
});

let dataChannel;
const pc = new RTCPeerConnection(configuration);

pc.onicecandidate = (event) => {
    if (event.candidate) {
        ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
    }
};

pc.ondatachannel = (event) => {
    dataChannel = event.channel;
    dataChannel.onopen = () => {
        console.log("Data channel opened");
    };
    dataChannel.onmessage = (event) => {
        console.log("Received message:", event.data);
    };
};

joyListener.subscribe((message) => {
    console.log('Received joystick data:', message);
    const joystickData = {
        axes: message.axes,
        buttons: message.buttons
    };
    document.getElementById("joystick-data").innerHTML = `
        <p>X축: ${message.axes[0].toFixed(2)}</p>
        <p>Y축: ${message.axes[1].toFixed(2)}</p>
        <p>Z축: ${message.axes[2].toFixed(2)}</p>
        <p>버튼 잡기(LB): ${message.buttons[0]}</p>
        <p>버튼 잡기(RB): ${message.buttons[1]}</p>
    `;
    if (dataChannel && dataChannel.readyState === "open") {
        dataChannel.send(JSON.stringify(joystickData));
    } else {
        console.log("Data channel is not open, state:", dataChannel ? dataChannel.readyState : "not created");
    }
});

ws.onmessage = async (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "answer") {
        await pc.setRemoteDescription(new RTCSessionDescription(message));
    } else if (message.type === "candidate") {
        await pc.addIceCandidate(new RTCIceCandidate(message.candidate));
    }
};

dataChannel = pc.createDataChannel("joystickData");
dataChannel.onopen = () => {
    console.log("Data channel opened");
};

const offer = await pc.createOffer();
await pc.setLocalDescription(offer);
ws.send(JSON.stringify(offer));