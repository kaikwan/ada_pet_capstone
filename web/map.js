/**
 * Setup all visualization elements when the page is loaded.
 */
function init_map() {
    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
        divID : 'map',
        width : 600,
        height : 500
    });

    // Setup the map client.
    var gridClient = new ROS2D.OccupancyGridClient({
        ros : ros,
        rootObject : viewer.scene
    });
    // Scale the canvas to fit to the map
    gridClient.on('change', function(){
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    });
}