const canvas = document.getElementById("display");
const ctx = canvas.getContext("2d");

const width = window.innerWidth;
const height = window.innerHeight

const boids = [];
const boidAmount = 10;
const maxBoidAmount = 500;

const boidMinSize = 2;
const boidMaxSize = 10;
const boidWanderRadius = 6;
const boidWanderSpeed = 40;
const boidSightRange = 150;
const boidSoftBoundFactor = (boidMaxSize + boidMinSize);
const boidAlignmentFactor = 0.01;
const boidCohesionFactor = 0.02;

const boidSeperationFactor = 80;
const boidSeperationDistance = 200;

// const boidMaxCharge = 100;
// const boidChargeShare = 250;
// const boidChargeGain = 40;
// const boidChargeTime = 2 * boidChargeGain; // double it -> seconds
// const boidChargeColor = "#FCF9D9";

let lastTime = 0;


function init() {
  canvas.width = width;
  canvas.height = height;

  for (let i = 0; i < boidAmount; i++) {
    boids.push(newBoid());
  }

  loop();
}

function loop() {
  ctx.fillStyle = "rgba(0, 0, 0, 0.025)";
  ctx.fillRect(0, 0, width, height);

  let time = performance.now();
  let deltaTimeMS = Math.min(time - lastTime, 20);
  let deltaTime = deltaTimeMS / 1000;
  lastTime = time;

  let fps = 1 / deltaTime;

  if (fps > 60 && boids.length < maxBoidAmount) {
    boids.push(newBoid());
  }

  if (fps < 50) {
    boids.pop();
  }

  ctx.font = "12px monospace";
  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, 6 * 12, 12);
  ctx.textBaseline = "top";
  ctx.fillStyle = "white";
  ctx.fillText("FPS: " + fps.toFixed(1), 0, 0);

  for (let i = 0; i < boids.length; i++) {
    updateBoid(i, deltaTime);
    drawBoid(i);
  }

  requestAnimationFrame(loop);
}

function updateBoid(index, dt) {
  const boid = boids[index];

  // update stuffs
  boid.wanderAngle += (Math.random() * 2 - 1) * boidWanderSpeed * dt;
  // if (boid.charge < -boidChargeTime) boid.charge = boidMaxCharge;
  // boid.charge -= boidChargeGain * dt;

  // do everything
  applyForces(index, dt);

  // collisions
  applyCollisions(index);

  boid.x += (Math.cos(boid.angle) * boid.speed + Math.cos(boid.wanderAngle) * boidWanderRadius) * dt;
  boid.y += (Math.sin(boid.angle) * boid.speed + Math.sin(boid.wanderAngle) * boidWanderRadius) * dt;
}

function applyForces(index, dt) {
  const boid0 = boids[index];
  let averageVel = { x: 0, y: 0 };
  let averagePos = { x: 0, y: 0 };
  let boidsSpotted = 0;
  let softBoundForce = { x: 0, y: 0 };

  let tooClose = { x: 0, y: 0 }
  let tooCloseCount = 0;

  if (boid0.x < boid0.size + boidSightRange) softBoundForce.x += boidSoftBoundFactor * (boidSightRange - boid0.x) / boidSightRange;
  if (boid0.x > width - (boid0.size + boidSightRange)) softBoundForce.x -= boidSoftBoundFactor * (boid0.x - (width - boidSightRange)) / boidSightRange;
  if (boid0.y < boid0.size + boidSightRange) softBoundForce.y += boidSoftBoundFactor * (boidSightRange - boid0.y) / boidSightRange;
  if (boid0.y > height - (boid0.size + boidSightRange)) softBoundForce.y -= boidSoftBoundFactor * (boid0.y - (height - boidSightRange)) / boidSightRange;

  for (let i = 0; i < boids.length; i++) {
    if (i == index) continue;
    const boid1 = boids[i];

    const dx = boid1.x - boid0.x;
    const dy = boid1.y - boid0.y;
    const distance = Math.sqrt(dx * dx + dy * dy);

    if (distance < boidSightRange + boid0.size + boid1.size) {
      // if (boid1.charge < 0) boid0.charge -= boidChargeShare * dt;

      averageVel.x += Math.cos(boid1.angle) * boid1.speed;
      averageVel.y += Math.sin(boid1.angle) * boid1.speed;
      averagePos.x += boid1.x;
      averagePos.y += boid1.y;

      boidsSpotted++;
    }

    if (distance < boidSeperationDistance + boid0.size + boid1.size) {
      tooCloseCount++;
      tooClose.x += boid1.x;
      tooClose.y += boid1.y;
    }
  }

  if (boidsSpotted == 0) return;

  averageVel.x /= boidsSpotted;
  averageVel.y /= boidsSpotted;
  averagePos.x /= boidsSpotted;
  averagePos.y /= boidsSpotted;

  if (tooCloseCount > 0) {
    tooClose.x /= tooCloseCount;
    tooClose.y /= tooCloseCount;
  }

  let seperationMagnitude = Math.sqrt(tooClose.x * tooClose.x + tooClose.y * tooClose.y);
  let seperationFactor = (boidSeperationFactor / seperationMagnitude) * (1 - boidSeperationDistance / seperationMagnitude);
  if (seperationMagnitude == 0) seperationFactor = 0;

  const newVel = {
    x: Math.cos(boid0.angle) * boid0.speed
      + averageVel.x * boidAlignmentFactor
      + (averagePos.x - boid0.x) * boidCohesionFactor
      + (boid0.x - tooClose.x) * seperationFactor
      + softBoundForce.x,
    y: Math.sin(boid0.angle) * boid0.speed
      + averageVel.y * boidAlignmentFactor
      + (averagePos.y - boid0.y) * boidCohesionFactor
      + (boid0.y - tooClose.y) * seperationFactor
      + softBoundForce.y
  };
  const magnitude = Math.sqrt(newVel.x * newVel.x + newVel.y * newVel.y);
  boid0.angle = Math.atan2(newVel.y / magnitude, newVel.x / magnitude);
}

function applyCollisions(index) {
  const boid0 = boids[index];

  for (let i = index + 1; i < boids.length; i++) {
    const boid1 = boids[i];
    const dx = boid1.x - boid0.x;
    const dy = boid1.y - boid0.y;
    const distance = Math.sqrt(dx * dx + dy * dy);

    if (distance < boid0.size + boid1.size) {
      const overlap = (boid0.size + boid1.size - distance) / 2;
      const ox = overlap * dx / distance;
      const oy = overlap * dy / distance;
      boid0.x -= ox;
      boid0.y -= oy;
      boid1.x += ox;
      boid1.y += oy;
    }
  }

  if (boid0.x < boid0.size) boid0.x = boid0.size;
  if (boid0.x > width - boid0.size) boid0.x = width - boid0.size;
  if (boid0.y < boid0.size) boid0.y = boid0.size;
  if (boid0.y > height - boid0.size) boid0.y = height - boid0.size;
}

function drawBoid(index) {
  const boid = boids[index];

  // if (boid.charge < 0) {
  //   ctx.fillStyle = boidChargeColor;

  //   ctx.shadowBlur = boid.size;
  //   ctx.shadowColor = boidChargeColor;

  //   ctx.beginPath();
  //   ctx.arc(boid.x, boid.y, boid.size, 0, Math.PI * 2);
  //   ctx.fill();

  //   ctx.shadowBlur = 0;
  //   return;
  // }

  ctx.fillStyle = boid.color;

  ctx.beginPath();
  ctx.arc(boid.x, boid.y, boid.size, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "white";

  ctx.beginPath();
  ctx.arc(boid.x + Math.cos(boid.angle) * boid.size, boid.y + Math.sin(boid.angle) * boid.size, boid.size / 4, 0, Math.PI * 2);
  ctx.fill();

  // ctx.strokeStyle = "white";
  // ctx.setLineDash([2, 12]);

  // ctx.beginPath();
  // ctx.arc(boid.x, boid.y, boidSeperationDistance + boid.size, 0, Math.PI * 2);
  // ctx.stroke();

  // ctx.beginPath();
  // ctx.arc(boid.x, boid.y, boidSightRange + boid.size, 0, Math.PI * 2);
  // ctx.stroke();

  // ctx.beginPath();
  // ctx.moveTo(boid.x, boid.y);
  // ctx.lineTo(boid.x + Math.cos(boid.angle) * boid.speed, boid.y + Math.sin(boid.angle) * boid.speed);
  // ctx.stroke();
}

function newBoid() {
  return {
    x: Math.random() * canvas.width,
    y: Math.random() * canvas.height,
    wanderAngle: Math.random() * Math.PI * 2,
    angle: Math.random() * Math.PI * 2,
    speed: Math.random() * 20 + 200,
    size: Math.random() * (boidMaxSize - boidMinSize) + boidMinSize,
    color: "hsl(" + Math.random() * 360 + ", 100%, 50%)",
    // charge: (boidMaxCharge + boidChargeTime) * Math.random() - boidChargeTime
  }
}

init();
