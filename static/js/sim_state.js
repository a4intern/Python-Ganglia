export const state = {
    ws: null,
    isConnected: false,
    chartWindowSec: 30,
    incomingBuffer: [],
    exportDataBuffer: [],
    chartDirty: false,
    currentActiveParams: {
        position: { c0: 0.5, c1: 0.0, c2: 0.1, d1: 1.0 },
        velocity: { c0: 0.1, c1: 0.01, c2: 0.0, d1: 1.0 },
        current: { c0: 1.0, c1: 0.1, c2: 0.0, d1: 1.0 }
    }
};

export const charts = {
    velChart: null,
    accChart: null,
    curChart: null,
    z1Chart: null,
    z2Chart: null,
    z3Chart: null
};

export const CONSTANTS = {
    CONTROL_PERIOD: 0.001,
    CONTROL_FREQ: 1000.0,
    OUTPUT_SCALE: 1.0
};
