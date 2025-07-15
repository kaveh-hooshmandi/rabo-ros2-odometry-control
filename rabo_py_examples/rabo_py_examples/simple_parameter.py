import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter



class SimpleParameter(Node):
    def __init__(self):

        super().__init__("simple_parameter")

        self.declare_parameter("_int_parameter",10)
        self.declare_parameter("_string_parameter","kaveh")

        self.add_on_set_parameters_callback(self.parameterChangeCallback)

        


    def  parameterChangeCallback(self,params):
        result = SetParametersResult()

        for param in params:
            if param.name == "_int_parameter" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Parametr _int_parameter changed  New value  is %d" %param.value)
                result.successful=True

            if param.name == "_string_parameter" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Parametr _string_parameter changed  New value  is %s" %param.value)
                result.successful=True

        return result
            
                   
            




def main():
    rclpy.init()
    simple_parameter=SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()


