// 집 환경: PC1 (192.168.35.2), PC2 (192.168.35.25)
const ws = new WebSocket("ws://192.168.35.2:8080");

// 직장 환경: PC1 (172.22.156.216), PC2 (172.17.3.215)
// const ws = new WebSocket("ws://172.22.156.216:8080");

const configuration = {
    iceServers: [
        { urls: "stun:stun.l.google.com:19302" },
        { urls: "turn:192.168.35.2:3478", username: "user", credential: "pass" }
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

// WebRTC 설정 및 연결
async function setupWebRTC() {
    try {
        // 데이터 채널 생성
        dataChannel = pc.createDataChannel("joystickData");
        dataChannel.onopen = () => {
            console.log("Data channel opened");
        };
        dataChannel.onclose = () => {
            console.log("Data channel closed, attempting reconnect...");
            setupWebRTC();
        };
        
        dataChannel.onmessage = (event) => {
            console.log("Received message:", event.data);
        };

        // ICE 후보 처리
        pc.onicecandidate = (event) => {
            if (event.candidate) {
                ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
            }
        };

        // 시그널링 서버 연결
        ws.onopen = () => {
            console.log("Connected to signaling server");
        };

        ws.onerror = (error) => {
            console.log("Signaling server error:", error);
        };

        ws.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            if (message.type === "answer") {
                await pc.setRemoteDescription(new RTCSessionDescription(message));
            } else if (message.type === "candidate") {
                await pc.addIceCandidate(new RTCIceCandidate(message.candidate));
            }
        };

        // WebRTC 오퍼 생성 및 전송
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        ws.send(JSON.stringify(offer));
    } catch (error) {
        console.error("Error setting up WebRTC:", error);
    }
}

// 조이스틱 데이터 처리
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

// WebRTC 설정 실행
setupWebRTC();