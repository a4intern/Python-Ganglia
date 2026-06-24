let ws;
let isConnected = false;
let chartWindowSec = 30;

const CONTROL_PERIOD = 0.001;
const CONTROL_FREQ = 1000.0;
const OUTPUT_SCALE = 1.0;

let currentActiveParams = {
    position: { c0: 0.5, c1: 0.0, c2: 0.1, d1: 1.0 },
    velocity: { c0: 0.1, c1: 0.01, c2: 0.0, d1: 1.0 },
    current: { c0: 1.0, c1: 0.1, c2: 0.0, d1: 1.0 }
};

let incomingBuffer = [];
let exportDataBuffer = [];
const maxExportDataPoints = 60000;
let chartDirty = false;

let renderDecimationCounter = 0;

let lastTime = null;
let lastVel = 0;
let smoothedAcc = 0;
const accAlpha = 0.05;

let aiChatDragDist = 0; 
let isDraggingAiChat = false; 
let aiChatStartX = 0; 
let aiChatStartLeft = 0;
