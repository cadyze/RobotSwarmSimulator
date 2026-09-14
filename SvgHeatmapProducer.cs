using System.Globalization;
using System.IO;

namespace RobotSwarmSimulator;

/// <summary>Creates a portable SVG visit heatmap from the CSV movement log.</summary>
public sealed class SvgHeatmapProducer
{
    public string Create(string movementsCsvPath, int size, (int X, int Y) target, string outputPath)
    {
        var visits = new int[size, size];
        foreach (var line in File.ReadLines(movementsCsvPath).Skip(1))
        {
            var cells = line.Split(',');
            if (cells.Length < 6 || !int.TryParse(cells[2], out var x) || !int.TryParse(cells[3], out var y)) continue;
            if (x >= 0 && x < size && y >= 0 && y < size) visits[x, y]++;
        }
        var maximum = Math.Max(1, visits.Cast<int>().Max());
        const double radius = 15, margin = 55;
        var horizontalStep = 1.5 * radius;
        var verticalStep = Math.Sqrt(3) * radius;
        var graphWidth = (size - 1) * horizontalStep + (size - 1) * horizontalStep / 2 + 2 * radius + 2 * margin;
        var graphHeight = (size - 1) * verticalStep + 2 * radius + 2 * margin;
        using var writer = new StreamWriter(outputPath);
        writer.WriteLine($"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{graphWidth.ToString(CultureInfo.InvariantCulture)}\" height=\"{graphHeight.ToString(CultureInfo.InvariantCulture)}\" viewBox=\"0 0 {graphWidth.ToString(CultureInfo.InvariantCulture)} {graphHeight.ToString(CultureInfo.InvariantCulture)}\">");
        writer.WriteLine("<rect width=\"100%\" height=\"100%\" fill=\"#fbfcfe\"/><text x=\"16\" y=\"28\" font-family=\"system-ui, sans-serif\" font-size=\"18\" font-weight=\"600\" fill=\"#1f2937\">Robot position visits</text><g stroke-linejoin=\"round\">");
        for (var y = 0; y < size; y++) for (var x = 0; x < size; x++)
        {
            var ratio = visits[x, y] / (double)maximum;
            var color = ViridisLike(ratio);
            var centerX = margin + radius + x * horizontalStep + y * horizontalStep / 2;
            var centerY = margin + radius + (size - 1 - y) * verticalStep;
            var outline = (x, y) == target ? "#dc2626" : "#ffffff";
            var width = (x, y) == target ? 3.5 : 1.25;
            writer.WriteLine($"<polygon points=\"{HexagonPoints(centerX, centerY, radius)}\" fill=\"{color}\" stroke=\"{outline}\" stroke-width=\"{width.ToString(CultureInfo.InvariantCulture)}\"><title>x={x}, y={y}, visits={visits[x, y]}</title></polygon>");
        }
        writer.WriteLine("</g></svg>");
        return outputPath;
    }

    private static string ViridisLike(double value)
    {
        var r = (int)Math.Round(68 + 185 * value); var g = (int)Math.Round(1 + 230 * value); var b = (int)Math.Round(84 - 47 * value);
        return $"rgb({r.ToString(CultureInfo.InvariantCulture)},{g.ToString(CultureInfo.InvariantCulture)},{b.ToString(CultureInfo.InvariantCulture)})";
    }

    private static string HexagonPoints(double centerX, double centerY, double radius) => string.Join(' ', Enumerable.Range(0, 6).Select(index =>
    {
        var angle = (30 + index * 60) * Math.PI / 180;
        return $"{(centerX + radius * Math.Cos(angle)).ToString("F2", CultureInfo.InvariantCulture)},{(centerY + radius * Math.Sin(angle)).ToString("F2", CultureInfo.InvariantCulture)}";
    }));
}
