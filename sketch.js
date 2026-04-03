

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
    let step_size = 0.0001;
    let gradLimit = 0.01;
    let rot_lim = 1;

    for (let k=0; k < iterLimit; k++) {
      phi = [...phi_prev];
      let grad = this.grad(phi_prev, step_size)
      for (let i=0; i < phi.length; i++) {
        phi[i] -= a * exp(-0.2 * k) * grad[i];
        if (phi[i] > rot_lim+i) {phi[i] = rot_lim;}
        if (phi[i] < -rot_lim-i) {phi[i] = -rot_lim;}
      }
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
      console.log(this._solution);
      
    }
  }

  render() {
    push();
    translate(this._origin.x, this._origin.y);

    for (let i=0; i<this._lengths.length; i++) {
      rotate(this._solution[i]);
      //strokeWeight((this._lengths.length-i)*15)
      stroke(220, 75, 75);
      line(0, 0, 0, this._lengths[i]);
      circle(0, 0, this._lengths[i]/2)
      stroke(360);
      let rect_wid = this._lengths[i]/4;
      rect(-rect_wid/2, 0, rect_wid, this._lengths[i], this._lengths[i]/20);
      strokeWeight(3);
      fill(100)
      circle(0, 0, this._lengths[i]/12);
      noFill()
      strokeWeight(1);
      stroke(11, 100, 68);
      if (i == this._lengths.length-1) {
      line(0, this._lengths[i], 0, this._lengths[i]+20);
      line(0, this._lengths[i], -20, this._lengths[i]);
      }
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
  createCanvas(970, 400);
  colorMode('hsb', 360, 100, 100);
  frameRate(60);

  test = new Linkage(createVector(3 * width/5, 0), [160, 110, 80, 30], [0, 0, 0, 0]);
}

function draw() {
  background(0);
  noFill();
  stroke(214, 76, 45);
  let set_point = createVector(mouseX, mouseY);
  test.update(set_point);
  test.render();
  
  circle(set_point.x, set_point.y, 5)
}


