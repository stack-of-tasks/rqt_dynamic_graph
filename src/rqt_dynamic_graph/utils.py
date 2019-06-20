from python_qt_binding import QT_BINDING_VERSION
if float(QT_BINDING_VERSION.split(".")[0]) < 5:
    # for retrocompatibility
    from python_qt_binding.QtGui import QGridLayout, QWidget
else:
    # for compatibility with pyqt5 or higher
    from python_qt_binding.QtWidgets import QGridLayout, QWidget

try:
    from dynamic_graph_manager.srv import RunCommand as ros_srv_RunCommand
    run_command_service_name = '/dynamic_graph/run_python_command'
except:
    from dynamic_graph_bridge_msgs.srv import RunCommand as ros_srv_RunCommand
    run_command_service_name = 'run_command'


def _build_common_widget(parent, nrow, ncol, *args, **kwargs):
    """Generate a QGridLayout widget made out of several widgets passed by *args.

    :param int nrow: number of rows of the widget.
    :param int ncol: number of columns of the widget.
    :returns: common widget
    :rtype: ``QWidget``
    """

    # Check size and number of widgets match
    if not (nrow * ncol == len(args)):
        raise RuntimeError("Trying to put %i widgets in a GridLayout with %i cells."
                           % (len(args), nrow * ncol))

    common_widget = QWidget(parent)
    common_layout = QGridLayout(common_widget)
    common_layout.setHorizontalSpacing(nrow)
    common_layout.setVerticalSpacing(ncol)

    # Set stretch factors
    if "stretch_columns" in kwargs:
        for col in range(ncol):
            common_layout.setColumnStretch(col, kwargs["stretch_columns"][col])

    for i in range(0, nrow):
        for j in range(0, ncol):
            index = j + i * ncol
            common_layout.addWidget(args[index], i, j)

    return common_widget