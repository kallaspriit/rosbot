import ros, { std_msgs } from "rclnodejs";

ros.init().then(() => {
  const node = new ros.Node("listener");

  node.createSubscription(
    "std_msgs/msg/String",
    "chat",
    {},
    (message: std_msgs.msg.String) => {
      console.log(`Received: ${message.data}`);
    }
  );

  node.spin();
});
