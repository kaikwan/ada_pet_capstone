const CLICK_CLICK_SPEEDS = {
slow: 0.05,
fast: 0.2,
};

const JOINT_LIMITS = {
yaw: { min: -1.57, max: 1.57 },
pitch: { min: -1.0, max: 1.0 },
roll: { min: -1.0, max: 1.0 },
};

const LIFT_ARM_LIMITS = {
lift: { min: 0.0, max: 1.0947933438242243 },
arm: { min: 0.0, max: 0.5154697810672271 },
};

const BASE_TRANSLATE_STEP = 0.1;  // meters
const BASE_ROTATE_STEP = 0.2;     // radians
