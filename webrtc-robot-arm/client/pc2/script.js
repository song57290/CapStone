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

const pc = new RTCPeerConnection(configuration);

// WebRTC 설정 및 연결
async function setupWebRTC() {
    try {
        // ICE 후보 처리
        pc.onicecandidate = (event) => {
            if (event.candidate) {
                console.log("PC2: Sending ICE candidate:", event.candidate);
                ws.send(JSON.stringify({ type: "candidate", candidate: event.candidate }));
            }
        };

        // 데이터 채널 수신
        pc.ondatachannel = (event) => {
            const dataChannel = event.channel;
            dataChannel.onopen = () => {
                console.log("PC2: Data channel opened");
            };
            dataChannel.onclose = () => {
                console.log("PC2: Data channel closed");
            };
            dataChannel.onmessage = (event) => {
                const joystickData = JSON.parse(event.data);
                console.log("PC2: Received joystick data:", joystickData);
                document.getElementById("joystick-data").innerHTML = `
                    <p>X축: ${joystickData.axes[0].toFixed(2)}</p>
                    <p>Y축: ${joystickData.axes[1].toFixed(2)}</p>
                    <p>Z축: ${joystickData.axes[2].toFixed(2)}</p>
                    <p>버튼 잡기(LB): ${joystickData.buttons[0]}</p>
                    <p>버튼 잡기(RB): ${joystickData.buttons[1]}</p>
                `;
            };
        };

        // 시그널링 서버 연결
        ws.onopen = () => {
            console.log("PC2: Connected to signaling server");
        };

        ws.onerror = (error) => {
            console.log("PC2: Signaling server error:", error);
        };

        ws.onclose = () => {
            console.log("PC2: Signaling server connection closed");
        };

        ws.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            console.log("PC2: Received message:", message);
            if (message.type === "offer") {
                console.log("PC2: Setting remote description");
                await pc.setRemoteDescription(new RTCSessionDescription(message));
                console.log("PC2: Creating answer");
                const answer = await pc.createAnswer();
                console.log("PC2: Setting local description");
                await pc.setLocalDescription(answer);
                console.log("PC2: Sending answer");
                ws.send(JSON.stringify(answer));
            } else if (message.type === "candidate") {
                console.log("PC2: Adding ICE candidate");
                await pc.addIceCandidate(new RTCIceCandidate(message.candidate));
            }
        };
    } catch (error) {
        console.error("PC2: Error setting up WebRTC:", error);
    }
}

// WebRTC 설정 실행
setupWebRTC();

// 새로고침/페이지 닫을 때 clean-up
window.addEventListener('beforeunload', () => {
  ws.close();
  pc.close();
});