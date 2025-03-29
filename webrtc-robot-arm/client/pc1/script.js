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

window.addEventListener("gamepadconnected", (e) => {
    console.log("Joystick connected:", e.gamepad);
    setInterval(() => {
        const gamepad = navigator.getGamepads()[e.gamepad.index];
        const joystickData = {
            axes: gamepad.axes,
            buttons: gamepad.buttons.map(b => b.pressed)
        };
        if (dataChannel.readyState === "open") {
            dataChannel.send(JSON.stringify(joystickData));
        }
    }, 100);
});
