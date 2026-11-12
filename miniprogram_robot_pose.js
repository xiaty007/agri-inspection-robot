// Copy into the mini program's utils directory. Uses a type="2d" Canvas context.
// meta must belong to the map PNG actually drawn, not a newer pending image.
function projectPose(pose, meta) {
  if (!pose || !pose.valid || !meta || !meta.available ||
      pose.frame_id !== meta.frame_id || !meta.origin) return null;
  const values = [pose.x, pose.y, pose.yaw, meta.origin.x, meta.origin.y,
    meta.origin.yaw, meta.resolution, meta.width, meta.height];
  if (!values.every(value => typeof value === 'number' && Number.isFinite(value)) ||
      meta.resolution <= 0 || meta.width <= 0 || meta.height <= 0) return null;
  const dx = pose.x - meta.origin.x;
  const dy = pose.y - meta.origin.y;
  const c = Math.cos(meta.origin.yaw);
  const s = Math.sin(meta.origin.yaw);
  const x = (c * dx + s * dy) / meta.resolution;
  const gridY = (-s * dx + c * dy) / meta.resolution;
  const angle = -(pose.yaw - meta.origin.yaw);
  return {
    x, y: meta.height - gridY,
    yaw: Math.atan2(Math.sin(angle), Math.cos(angle)),
    inBounds: x >= 0 && x < meta.width && gridY >= 0 && gridY < meta.height
  };
}

// box is the bounding rectangle in which the map PNG is fitted uniformly.
// Draw the PNG with exactly this same scale and centering before calling this.
function drawRobot(ctx, pose, meta, box) {
  const pixel = projectPose(pose, meta);
  if (!pixel || !pixel.inBounds || !box ||
      ![box.x, box.y, box.width, box.height].every(Number.isFinite) ||
      box.width <= 0 || box.height <= 0) return false;
  const scale = Math.min(box.width / meta.width, box.height / meta.height);
  const left = box.x + (box.width - meta.width * scale) / 2;
  const top = box.y + (box.height - meta.height * scale) / 2;
  ctx.save();
  ctx.translate(left + pixel.x * scale, top + pixel.y * scale);
  ctx.rotate(pixel.yaw);
  ctx.fillStyle = '#1677ff';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(12, 0); // Arrow points along the robot's +X (forward).
  ctx.lineTo(-8, -7);
  ctx.lineTo(-5, 0);
  ctx.lineTo(-8, 7);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
  return true;
}

module.exports = { projectPose, drawRobot };
