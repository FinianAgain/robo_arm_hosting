

class Linkage {
  constructor(origin, lengths, startAngles) {
    this._origin = origin;
    this._lengths = lengths;
    this._solution = startAngles;
    this._prevSolution;
    this._target = createVector(0, 0);
  }

  f(angles) {  // Find end position using forward kinematics
    let x = this._origin.x;
    let y = this._origin.y;
    for (let i=0; i < this._lengths.length; i++) {
      x += -1 * this._lengths[i] * sin(angles[i]);
      y += this._lengths[i] * cos(angles[i]);
    }
    return createVector(x, y);
  }

  G(angles) {  // Find current error
    let f_k = this.f(angles);
    return this._target.dist(f_k);
  }

  grad(angles, step_size) {  // approx. gradient using center diff
    let grad = [];
    for (let i=0; i < angles.length ; i++) {
      let a_pos = [...angles];
      let a_neg = [...angles];
      a_pos[i] += step_size;
      a_neg[i] -= step_size;
      grad.push((this.G(a_pos) - this.G(a_neg)) / (2 * step_size));

    }
    return grad;
  }

  gradDescent() {
    let phi;
    let phi_prev = [...this._prevSolution];
    let a = 0.01;
    let iterLimit = 120;
    let step_size = 0.0001
    let gradLimit = 0.01

    for (let k=0; k < iterLimit; k++) {
      phi = [...phi_prev];
      let grad = this.grad(phi_prev, step_size)
      for (let i=0; i < phi.length; i++) {phi[i] -= a * exp(-0.2 * k) * grad[i];}
      if (arr_mag(grad) <= gradLimit) {break;}
      phi_prev = phi;
    }
    return phi;

  }

  update(newTarget) {
    if (!this._target.equals(newTarget)) {
      this._target = newTarget;
      this._prevSolution = this._solution;
      this._solution = this.gradDescent();
    }
  }

  render() {
    push();
    translate(this._origin.x, this._origin.y);

    for (let i=0; i<this._lengths.length; i++) {
      rotate(this._solution[i]);
      strokeWeight((this._lengths.length-i)*15)
      line(0, 0, 0, this._lengths[i]);
      translate(0, this._lengths[i]);
      rotate(-1 * this._solution[i]);
    }
    pop();
  }
}


function arr_mag(array) {
  let cnt = 0
  for (let i = 0; i < array.length; i++) {
    cnt += array[i]**2;
  }
  return sqrt(cnt);
}
  
function setup() {
  createCanvas(400, 400);
  colorMode('hsb');
  frameRate(60);

  test = new Linkage(createVector(width/2, height), [200, 125, 125, 50], [0, 0, 0, 0]);
}

function draw() {
  background(10);
  noFill();
  stroke(220, 75, 75);
  let set_point = createVector(mouseX, mouseY);
  test.update(set_point);
  test.render();
}


