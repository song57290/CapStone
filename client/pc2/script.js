const configuration = {
    iceServers: [
        { urls: "stun:stun.l.google.com:19302" }
    ]
};
const peerConnection = new RTCPeerConnection(configuration);
const ws = new WebSocket("ws://localhost:8080");

peerConnection.ondatachannel = (event) => {
    const dataChannel = event.channel;
    dataChannel.onmessage = (event) => {
        const joystickData = JSON.parse(event.data);
        console.log("Received joystick data:", joystickData);
        // HTML에 데이터 표시
        document.getElementById("joystick-data").innerHTML = `
            <p>X축: ${joystickData.axes[0].toFixed(2)}</p>
            <p>Y축: ${joystickData.axes[1].toFixed(2)}</p>
            <p>Z축: ${joystickData.axes[2].toFixed(2)}</p>
            <p>잡기 버튼(A): ${joystickData.buttons[0]}</p>
            <p>놓기 버튼(B): ${joystickData.buttons[1]}</p>
        `;
    };
};

ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "offer") {
        peerConnection.setRemoteDescription(new RTCSessionDescription(message.sdp))
            .then(() => peerConnection.createAnswer())
            .then(answer => peerConnection.setLocalDescription(answer))
            .then(() => {
                ws.send(JSON.stringify({ type: "answer", sdp: peerConnection.localDescription }));
            });
    } else if (message.type === "candidate") {
        peerConnection.addIceCandidate(new RTCIceCandidate(message.candidate));
    }
};

peerConnection.onicecandidate = (event) => {
    if (event.candidate) {
        ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
    }
};