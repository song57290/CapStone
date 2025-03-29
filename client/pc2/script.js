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

const pc = new RTCPeerConnection(configuration);

pc.onicecandidate = (event) => {
    if (event.candidate) {
        ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
    }
};

pc.ondatachannel = (event) => {
    const dataChannel = event.channel;
    dataChannel.onmessage = (event) => {
        const joystickData = JSON.parse(event.data);
        console.log("Received joystick data:", joystickData);
        document.getElementById("joystick-data").innerHTML = `
            <p>X축: ${joystickData.axes[0].toFixed(2)}</p>
            <p>Y축: ${joystickData.axes[1].toFixed(2)}</p>
            <p>Z축: ${joystickData.axes[2].toFixed(2)}</p>
            <p>버튼 잡기(LB): ${joystickData.buttons[0]}</p>
            <p>버튼 잡기(RB): ${joystickData.buttons[1]}</p>
        `;
    };
};

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