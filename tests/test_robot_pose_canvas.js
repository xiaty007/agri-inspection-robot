const assert = require('node:assert/strict');
const { projectPose, drawRobot } = require('../miniprogram_robot_pose.js');

const meta = {
  available: true, frame_id: 'map', width: 100, height: 80,
  resolution: 0.1, origin: { x: -1, y: -2, yaw: 0 }
};
const pose = { valid: true, frame_id: 'map', x: 1, y: 1, yaw: Math.PI / 2 };
const pixel = projectPose(pose, meta);
assert.equal(pixel.x, 20);
assert.equal(pixel.y, 50);
assert.equal(pixel.yaw, -Math.PI / 2);
assert.equal(pixel.inBounds, true);
const rotated = projectPose(pose, {
  ...meta, origin: { x: 1, y: 0, yaw: Math.PI / 2 }
});
assert.ok(Math.abs(rotated.x - 10) < 1e-10);
assert.ok(Math.abs(rotated.y - 80) < 1e-10);
assert.ok(Math.abs(rotated.yaw) < 1e-10);
assert.equal(projectPose({ ...pose, valid: false }, meta), null);
assert.equal(projectPose({ ...pose, frame_id: 'odom' }, meta), null);
assert.equal(projectPose({ ...pose, x: NaN }, meta), null);
assert.equal(projectPose({ ...pose, x: null }, meta), null);
assert.equal(projectPose(pose, { ...meta, resolution: 0 }), null);
assert.equal(projectPose({ ...pose, x: 100 }, meta).inBounds, false);
const calls = [];
const ctx = {};
for (const name of ['save', 'translate', 'rotate', 'beginPath', 'moveTo', 'lineTo',
  'closePath', 'fill', 'stroke', 'restore']) {
  ctx[name] = (...args) => calls.push([name, ...args]);
}
const box = { x: 10, y: 20, width: 200, height: 200 };
assert.equal(drawRobot(ctx, pose, meta, box), true);
// Image is 200x160 centered inside the 200x200 box.
assert.deepEqual(calls.find(call => call[0] === 'translate'), ['translate', 50, 140]);
assert.deepEqual(calls.find(call => call[0] === 'rotate'), ['rotate', -Math.PI / 2]);
calls.length = 0;
assert.equal(drawRobot(ctx, { ...pose, valid: false }, meta, box), false);
assert.equal(calls.length, 0);
assert.equal(drawRobot(ctx, pose, meta, { ...box, width: 0 }), false);
console.log('Canvas pose projection and arrow drawing checks passed');
