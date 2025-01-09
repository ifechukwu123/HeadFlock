let flock = [];
let numBoids = 300;
let video;
let poseNet;
let poses = [];
let colourPalette = [
	[71, 13, 33],
	[156, 15, 72],
	[214, 125, 62],
	[255, 230, 27],
];

function setup() {
	createCanvas(500, 500);

	for (let i = 0; i < numBoids; i++) {
		let boid = new Boid(width / 2, height / 2, random(TWO_PI));
		boid.colour = random(colourPalette);
		flock.push(boid);
	}

	video = createCapture(VIDEO);
	video.size(width, height);

	poseNet = ml5.poseNet(video, modelReady);
	poseNet.on("pose", function (results) {
		poses = results;
	});

	video.hide();
}

function draw() {
	background(0, 20);
	flock.forEach((boid) => {
		boid.flock(flock);
		boid.update();
		stroke(boid.colour);
		strokeWeight(12);
		line(
			boid.position.x,
			boid.position.y,
			boid.oldPosition.x,
			boid.oldPosition.y
		);
	});

	fleeCameraMotion();
}

class Boid {
	constructor(x, y, angle) {
		this.acceleration = createVector(0, 0);
		this.velocity = createVector(cos(angle), sin(angle));
		this.position = createVector(x, y);
		this.oldPosition = createVector(0, 0);
		this.dimension = 4;
		this.maxspeed = 2.1;
		this.maxforce = 0.01;
		this.colour = [0, 0, 0];
	}

	applyForce(force) {
		this.acceleration.add(force);
	}

	flock(boids) {
		//the most important forces to adhere to in a flock
		let separation = this.separate(boids);
		let alignment = this.align(boids);
		let cohesion = this.cohesion(boids);

		separation.mult(1.5);

		this.applyForce(separation);
		this.applyForce(alignment);
		this.applyForce(cohesion);
	}

	update() {
		this.velocity.add(this.acceleration);
		this.velocity.limit(this.maxspeed);
		this.oldPosition = this.position;
		this.position.add(this.velocity);
		this.acceleration.mult(0);

		if (this.position.x < -this.dimension) {
			this.position.x = width + this.dimension;
		}
		if (this.position.y < -this.dimension) {
			this.position.y = height + this.dimension;
		}
		if (this.position.x > width + this.dimension) {
			this.position.x = -this.dimension;
		}
		if (this.position.y > height + this.dimension) {
			this.position.y = -this.dimension;
		}
	}

	seek(target) {
		let desired = p5.Vector.sub(target, this.position)
			.normalize()
			.mult(this.maxspeed);

		let steerForce = p5.Vector.sub(desired, this.velocity);
		steerForce.limit(this.maxforce);
		return steerForce;
	}

	separate(boids) {
		let desiredSeparation = 25.0;
		let steerForce = createVector(0, 0);
		let count = 0;

		// For every boid in the system, check if it's too close
		boids.forEach((boid) => {
			let d = p5.Vector.dist(this.position, boid.position);

			if (d > 0 && d < desiredSeparation) {
				let diff = p5.Vector.sub(this.position, boid.position).normalize();
				diff.div(d);
				steerForce.add(diff);
				count++; // Keep track of how many
			}
		});

		if (count > 0) {
			steerForce.div(count);
		}

		if (steerForce.mag() > 0) {
			steerForce.normalize();
			steerForce.mult(this.maxspeed);
			steerForce.sub(this.velocity);
			steerForce.limit(this.maxforce);
		}
		return steerForce;
	}

	align(boids) {
		let neighbordist = 50;
		let sum = createVector(0, 0);
		let count = 0;

		boids.forEach((boid) => {
			let d = p5.Vector.dist(this.position, boid.position);
			if (d > 0 && d < neighbordist) {
				sum.add(boid.velocity);
				count++;
			}
		});

		if (count > 0) {
			sum.div(count);

			sum.normalize();
			sum.mult(this.maxspeed);
			let steerForce = p5.Vector.sub(sum, this.velocity).limit(this.maxforce);
			return steerForce;
		} else {
			return createVector(0, 0);
		}
	}

	cohesion(boids) {
		let neighbordist = 50;
		let sum = createVector(0, 0);
		let count = 0;

		boids.forEach((boid) => {
			let d = p5.Vector.dist(this.position, boid.position);
			if (d > 0 && d < neighbordist) {
				sum.add(boid.position);
				count++;
			}
		});

		if (count > 0) {
			sum.div(count);
			return this.seek(sum); // Steer towards the position
		} else {
			return createVector(0, 0);
		}
	}

	flee(target) {
		let vec = p5.Vector.sub(this.position, target);

		if (abs(vec.x) > width / 2) {
			vec.x = this.position.x < target.x ? vec.x - width : vec.x + width;
		}

		if (abs(vec.y) > height / 2) {
			vec.y = this.position.y < target.y ? vec.y - height : vec.y + height;
		}

		const speed = constrain(
			map(vec.magSq(), 0, 1000, 0, this.maxspeed),
			0,
			this.maxspeed
		);

		if (vec.magSq() > 1000) {
		} else {
			vec.normalize().mult(speed);
			const steeringForce = vec.sub(this.velocity);
			this.applyForce(steeringForce);
		}
	}
}

function modelReady() {
	console.log("Model Loaded!");
}

function fleeCameraMotion() {
	// Loop through all the poses detected
	for (let i = 0; i < poses.length; i++) {
		let pose = poses[i].pose;
		for (let j = 0; j < pose.keypoints.length; j++) {
			let keypoint = pose.keypoints[j];
			if (keypoint.score > 0.2) {
				flock.forEach((boid) => {
					boid.flee(createVector(keypoint.position.x, keypoint.position.y));
				});
			}
		}
	}
}
