import ros from "rclnodejs";

ros.init().then(() => {
  const node = new ros.Node("talker");
  const publisher = node.createPublisher("std_msgs/msg/String", "chat");

  let count = 0;

  setInterval(() => {
    const message = `Hello #${count++} from nodejs!`;

    publisher.publish(message);

    console.log(`Published: ${message}`);
  }, 1000);

  node.spin();
});
