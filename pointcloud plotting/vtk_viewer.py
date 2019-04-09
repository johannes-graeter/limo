import argparse
import vtk


def main(filename):
    # Read and display
    reader = vtk.vtkPLYReader()
    reader.SetFileName(filename)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer = vtk.vtkRenderer()
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window_interactor = vtk.vtkRenderWindowInteractor()
    render_window_interactor.SetRenderWindow(render_window)

    renderer.AddActor(actor)
    renderer.SetBackground(0.1804, 0.5451, 0.3412)
    renderer.SetBackground(0.1804, 0.5451, 0.3412)

    render_window.Render()
    render_window_interactor.Start()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file", type=str, help="path to file")

    args = parser.parse_args()
    main(args.file)
