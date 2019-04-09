import numpy as np
import vtk
from vtk.util import numpy_support


def render(pcl):
    renderer = vtk.vtkRenderer()
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window_interactor = vtk.vtkRenderWindowInteractor()
    render_window_interactor.SetRenderWindow(render_window)

    print(pcl.height_min, pcl.height_max)
    renderer.AddActor(pcl.get_actor())

    render_window.Render()
    render_window_interactor.Start()


class VtkPointCloud:
    def __init__(self, color_mode="y"):
        self.color_axis = None
        if color_mode == "x":
            self.color_axis = 0
        if color_mode == "y":
            self.color_axis = 1
        if color_mode == "z":
            self.color_axis = 2

        self.is_color_mode_height = False
        if self.color_axis:
            self.is_color_mode_height = True

        self.vtk_poly_data = vtk.vtkPolyData()
        self.clear_points()

        self.point_ids = []
        self.color_ids = []

        self.height_min = 1234567
        self.height_max = -1234567

    def set_height(self, height_min, height_max, use_median=False):
        self.height_min = height_min
        self.height_max = height_max

        if use_median:
            med = self._get_median()
            self.height_min = med[self.color_axis] + height_min
            self.height_max = med[self.color_axis] + height_max
            print(self.height_min)

    def _get_median(self):
        points = numpy_support.vtk_to_numpy(self.vtk_points.GetData())
        med = np.median(np.transpose(points), 1)
        return med

    def get_actor(self, point_size=1):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtk_poly_data)
        mapper.SetColorModeToDefault()

        if self.is_color_mode_height:
            mapper.SetScalarRange(self.height_min, self.height_max)
            mapper.SetScalarVisibility(1)

        vtk_actor = vtk.vtkActor()
        vtk_actor.GetProperty().SetPointSize(point_size)
        vtk_actor.SetMapper(mapper)

        return vtk_actor

    def add_points(self, points, colors=None):
        if not colors:
            for p in points:
                self.add_point(p)
        elif np.array(colors).shape[0] == 3:
            for p in points:
                self.add_point(p, colors)
        else:
            for p, c in zip(points, colors):
                self.add_point(p, colors)

    def add_point(self, point, color=None):
        if not self.is_color_mode_height and color is None:
            raise Exception("Wrong input for color mode {}".format(self.color_mode))
        point_id = self.vtk_points.InsertNextPoint(point[:3])
        self.vtk_cells.InsertNextCell(1)
        self.vtk_cells.InsertCellPoint(point_id)
        self.point_ids.append(point_id)

        if color is not None:
            color_id = self.color_data.InsertNextTuple3(color[0], color[1], color[2])
        else:
            color_id = self.color_data.InsertNextValue(point[self.color_axis])
            self.height_min = min(self.height_min, point[self.color_axis])
            self.height_max = max(self.height_max, point[self.color_axis])

        self.color_ids.append(color_id)

        self.vtk_cells.Modified()
        self.vtk_points.Modified()
        self.color_data.Modified()

    def write(self, file_name):
        writer = vtk.vtkPLYWriter()
        writer.SetFileName(file_name)
        writer.SetInputData(self.vtk_poly_data)
        if self.is_color_mode_height:
            # set lookup tbale for depth values to colors
            lut = vtk.vtkLookupTable()
            lut.SetTableRange(self.height_min, self.height_max)
            lut.Build()
            # in order to be convertable to pcd, use lut to generate colors.
            # THIS IS A DIRTY HACK BUT NEEDED SINCE PCL IS STUPID
            # writer.SetLookupTable(lut) only works for meshlab
            cur_color_data = vtk.vtkUnsignedCharArray()
            cur_color_data.SetNumberOfComponents(3)
            for id in self.color_ids:
                val = self.color_data.GetValue(id)
                col = [0., 0., 0.]
                lut.GetColor(val, col)
                col = [int(c * 255) for c in col]
                cur_color_data.InsertNextTuple3(col[0], col[1], col[2])

            self.color_data = cur_color_data
            self.color_data.SetName("Colors")
            self.vtk_poly_data.GetPointData().SetActiveScalars('Colors')
            self.vtk_poly_data.GetPointData().SetScalars(self.color_data)

        writer.SetArrayName("Colors")
        writer.Write()

    def clear_points(self):
        self.vtk_points = vtk.vtkPoints()
        self.vtk_cells = vtk.vtkCellArray()

        self.vtk_poly_data.SetPoints(self.vtk_points)
        self.vtk_poly_data.SetVerts(self.vtk_cells)

        if self.is_color_mode_height:
            self.color_data = vtk.vtkDoubleArray()
            self.color_data.SetName('DepthArray')
            self.vtk_poly_data.GetPointData().SetActiveScalars('DepthArray')
        else:
            self.color_data = vtk.vtkUnsignedCharArray()
            self.color_data.SetNumberOfComponents(3)
            self.color_data.SetName("Colors")
            self.vtk_poly_data.GetPointData().SetActiveScalars('Colors')
        self.vtk_poly_data.GetPointData().SetScalars(self.color_data)
