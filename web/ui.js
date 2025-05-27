let isJoystickActive = false; // Tracks if the joystick is active
let joystickInterval = null; // Interval for sending continuous commands
let lastJoystickData = null; // Stores the last joystick data

// Ensure the slider and dropdown are reset correctly on page reload
document.addEventListener("DOMContentLoaded", () => {
  const speedSelect = document.getElementById("speedSelect");
  const jointSlider = document.getElementById("jointSlider");
  const jointValue = document.getElementById("jointValue");
  const jointSelect = document.getElementById("jointSelect");
  const controlSelect = document.getElementById("controlSelect");
  const controlSlider = document.getElementById("controlSlider");
  const controlValue = document.getElementById("controlValue");

  // Reset the dropdown to its default value
  speedSelect.value = "slow";
  updateJointSpeed(speedSelect.value);

  // Reset the dropdown to its default value
  jointSelect.value = "yaw";
  selectedJoint = "yaw";
  updateSliderForSelectedJoint();

  // Reset the control selection to its default value
  controlSelect.value = "lift";
  selectedControl = "lift";
  updateSliderForSelectedControl();

  // Track when the slider is being controlled
  jointSlider.addEventListener("mousedown", () => {
    isSliderActive = true;
  });

  jointSlider.addEventListener("mouseup", () => {
    isSliderActive = false;
  });

  jointSlider.addEventListener("mouseleave", () => {
    isSliderActive = false;
  });

  // Update the displayed value of the slider when it changes
  jointSlider.addEventListener("input", function () {
    jointValue.textContent = this.value;
    moveSelectedJointToAbsolute(parseFloat(this.value));
  });

  // Update the slider when the dropdown selection changes
  jointSelect.addEventListener("change", function () {
    selectedJoint = this.value;
    updateSliderForSelectedJoint();
  });

  // Track when the slider is being controlled
  controlSlider.addEventListener("mousedown", () => {
    isSliderActive = true;
  });

  controlSlider.addEventListener("mouseup", () => {
    isSliderActive = false;
  });

  controlSlider.addEventListener("mouseleave", () => {
    isSliderActive = false;
  });

  // Update the displayed value of the slider when it changes
  controlSlider.addEventListener("input", function () {
    controlValue.textContent = this.value;
    moveSelectedControlToAbsolute(parseFloat(this.value));
  });

  // Update the slider when the dropdown selection changes
  controlSelect.addEventListener("change", function () {
    selectedControl = this.value;
    updateSliderForSelectedControl();
  });

  initializeJoystick();
  updateRunstopButton();

  // Tab switching functionality
  const tabButtons = document.querySelectorAll(".tab-button");
  const tabPanes = document.querySelectorAll(".tab-pane");

  tabButtons.forEach((button) => {
    button.addEventListener("click", () => {
      // Remove active class from all buttons
      tabButtons.forEach((btn) => btn.classList.remove("active"));

      // Add active class to the clicked button
      button.classList.add("active");

      // Hide all tab panes
      tabPanes.forEach((pane) => pane.classList.remove("active"));

      // Show the corresponding tab pane
      const tabId = button.getAttribute("data-tab");
      document.getElementById(tabId).classList.add("active");
    });
  });

  // Create the main viewer.
  var viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 600,
      height : 500
  });

  // Log cursor coordinates on map click and draw direction vector
  let clickPoints = [];
  const mapDiv = document.getElementById('map');
  mapDiv.addEventListener('click', function(event) {
    // Get bounding rect for offset
    const rect = mapDiv.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Convert to map coordinates
    const mapCoords = viewer.scene.globalToRos(x, y);

    clickPoints.push({ x: mapCoords.x, y: -mapCoords.y }); // WARN: Not sure why y is negative here

    // If two points are clicked, draw the vector
    if (clickPoints.length === 2) {
      const [start, end] = clickPoints;
      if (window.vectorShape) {
        viewer.scene.removeChild(window.vectorShape);
      }

      // Display the vector direction
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      
      // Create an arrow shape to represent the normalized direction vector
      const arrow = new createjs.Shape();
      const normLength = 1.0;
      const angle = Math.atan2(dy, dx);
      const nx = start.x + normLength * Math.cos(angle);
      const ny = start.y + normLength * Math.sin(angle);

      // Draw the main normalized line with a thin stroke
      arrow.graphics
        .setStrokeStyle(0.1)
        .beginStroke("green")
        .moveTo(start.x, start.y)
        .lineTo(nx, ny)
        .endStroke();

      // Draw arrowhead with a thicker stroke for visibility
      const arrowSize = 0.5;
      arrow.graphics
        .setStrokeStyle(0.1)
        .beginStroke("green")
        .moveTo(nx, ny)
        .lineTo(
          nx - arrowSize * Math.cos(angle - Math.PI / 6),
          ny - arrowSize * Math.sin(angle - Math.PI / 6)
        )
        .moveTo(nx, ny)
        .lineTo(
          (nx - arrowSize * Math.cos(angle + Math.PI / 6)),
          (ny - arrowSize * Math.sin(angle + Math.PI / 6))
        )
        .endStroke();

      viewer.scene.addChild(arrow);
      window.vectorShape = arrow; // Store for later removal

      // Reset for next vector
      clickPoints = [];
    }
  });

  // Setup the map client.
  var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
  });
  // Scale the canvas to fit to the map
  gridClient.on('change', function(){
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
      viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
  });
});

// Function to initialize the joystick
const initializeJoystick = () => {
  const joystickContainer = document.getElementById("joystickContainer");

  // Destroy any existing joystick instance to prevent duplicates
  if (joystickInstance) {
    joystickInstance.destroy();
    joystickInstance = null;
  }

  // Create a new joystick instance with improved responsiveness
  joystickInstance = nipplejs.create({
    zone: joystickContainer,
    mode: "static", // Static mode keeps the joystick in a fixed position
    position: { left: "50%", top: "50%" }, // Center the joystick nipple
    color: "blue", // Set the color of the joystick
    size:
      Math.min(joystickContainer.clientWidth, joystickContainer.clientHeight) *
      0.8, // Responsive size
    dynamicPage: true, // Better handle page resize events
    lockX: false, // Allow X-axis movement
    lockY: false, // Allow Y-axis movement
  });

  // Handle joystick movement
  joystickInstance.on("move", (evt, data) => {
    lastJoystickData = data; // Store the last joystick data
    if (!isJoystickActive) {
      isJoystickActive = true;
      startJoystickCommandLoop();
    }
  });

  // Stop the base when the joystick is released
  joystickInstance.on("end", () => {
    console.log("Joystick released, stopping movement.");
    isJoystickActive = false;
    stopJoystickCommandLoop();
    executeFollowJointTrajectory(
      ["translate_mobile_base", "rotate_mobile_base"],
      [0, 0]
    );
  });
};

// Function to handle joystick movement
const handleJoystickMove = (data) => {
  if (data.direction) {
    const direction = data.direction.angle; // 'up', 'down', 'left', 'right'
    const distance = data.distance; // Distance from the center of the joystick
    const speed = Math.min(distance / 50, 2); // Normalize speed (0 to 1)

    if (direction === "up") {
      executeFollowJointTrajectory(["translate_mobile_base"], [speed * 0.1]); // Move forward
    } else if (direction === "down") {
      executeFollowJointTrajectory(["translate_mobile_base"], [-speed * 0.1]); // Move backward
    } else if (direction === "left") {
      executeFollowJointTrajectory(["rotate_mobile_base"], [speed * 0.1]); // Rotate left
    } else if (direction === "right") {
      executeFollowJointTrajectory(["rotate_mobile_base"], [-speed * 0.1]); // Rotate right
    }
    console.log(`Joystick moved: ${direction} with speed: ${speed}`);
  }
};

const startJoystickCommandLoop = () => {
  if (!joystickInterval) {
    joystickInterval = setInterval(() => {
      if (lastJoystickData) {
        handleJoystickMove(lastJoystickData); // Use the last known joystick data
      }
    }, 100); // Send commands every 100ms (adjust as needed)
  }
};

const stopJoystickCommandLoop = () => {
  clearInterval(joystickInterval);
  joystickInterval = null;
};

// Start continuous movement for the selected joint
const startMoveSelectedJoint = (direction) => {
  if (!jointInterval) {
    jointInterval = setInterval(() => moveSelectedJoint(direction), 100); // Adjust interval time as needed
  }
};

// Stop continuous movement for the selected joint
const stopMoveSelectedJoint = () => {
  clearInterval(jointInterval);
  jointInterval = null;
};
// Start continuous movement for the selected control
const startMoveSelectedControl = (direction) => {
  if (!jointInterval) {
    jointInterval = setInterval(() => moveSelectedControl(direction), 100); // Adjust interval time as needed
  }
};

// Stop continuous movement for the selected control
const stopMoveSelectedControl = () => {
  clearInterval(jointInterval);
  jointInterval = null;
};

// Update the slider and displayed value based on the selected joint
const updateSliderForSelectedJoint = () => {
  const slider = document.getElementById("jointSlider");
  const sliderValue = document.getElementById("jointValue");
  const limits = JOINT_LIMITS[selectedJoint];

  slider.min = limits.min;
  slider.max = limits.max;
  slider.value = currentJointPositions[selectedJoint].toFixed(2);
  sliderValue.textContent = slider.value;
};

// Update the slider and displayed value based on the selected control
const updateSliderForSelectedControl = () => {
  const slider = document.getElementById("controlSlider");
  const sliderValue = document.getElementById("controlValue");
  const limits = LIFT_ARM_LIMITS[selectedControl];

  slider.min = limits.min;
  slider.max = limits.max;
  slider.value = currentControlPositions[selectedControl].toFixed(2);
  sliderValue.textContent = slider.value;
};

// Function to update the toggle button's text and style
const updateRunstopButton = () => {
  const runstopButton = document.getElementById("runstopButton");
  if (runstopState) {
    runstopButton.textContent = "Runstop ON";
    runstopButton.style.backgroundColor = "red";
    runstopButton.style.color = "white";
  } else {
    runstopButton.textContent = "Runstop OFF";
    runstopButton.style.backgroundColor = "yellow";
    runstopButton.style.color = "black";
  }
};

// Handle window resize events to reinitialize the joystick
window.addEventListener("resize", () => {
  // Debounce the resize event to prevent too many reinitializations
  clearTimeout(window.resizeTimer);
  window.resizeTimer = setTimeout(() => {
    console.log("Window resized, reinitializing joystick");
    initializeJoystick();
  }, 250);
});
