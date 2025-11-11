import streamlit as st
import folium
from streamlit_folium import folium_static
import heapq
import time
from math import radians, cos, sin, asin, sqrt
import pandas as pd

# Configuración de la página
st.set_page_config(
    page_title="Optimización de Rutas - Cuenca",
    page_icon="🗺️",
    layout="wide"
)

# Definición de puntos de interés en Cuenca
CUENCA_NODES = {
    'Parque Calderon': {'lat': -2.8973, 'lng': -79.0047, 'id': 0},
    'Catedral Nueva': {'lat': -2.8969, 'lng': -79.0050, 'id': 1},
    'Museo Pumapungo': {'lat': -2.9067, 'lng': -79.0047, 'id': 2},
    'Mirador de Turi': {'lat': -2.9292, 'lng': -78.9947, 'id': 3},
    'Rio Tomebamba': {'lat': -2.9017, 'lng': -79.0000, 'id': 4},
    'Terminal Terrestre': {'lat': -2.9200, 'lng': -78.9850, 'id': 5},
    'Universidad de Cuenca': {'lat': -2.8850, 'lng': -79.0050, 'id': 6},
    'Mercado 10 de Agosto': {'lat': -2.8930, 'lng': -79.0080, 'id': 7},
    'Barranco del Tomebamba': {'lat': -2.9040, 'lng': -79.0030, 'id': 8},
    'Plaza de las Flores': {'lat': -2.8978, 'lng': -79.0042, 'id': 9}
}

def haversine(lat1, lon1, lat2, lon2):
    """
    Calcula la distancia en kilómetros entre dos puntos en la Tierra
    usando la fórmula de Haversine (heurística para A*)
    """
    R = 6371  # Radio de la Tierra en kilómetros
    
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    
    return R * c

def calculate_distance(node1, node2):
    """Calcula la distancia heurística entre dos nodos"""
    lat1 = CUENCA_NODES[node1]['lat']
    lng1 = CUENCA_NODES[node1]['lng']
    lat2 = CUENCA_NODES[node2]['lat']
    lng2 = CUENCA_NODES[node2]['lng']
    
    return haversine(lat1, lng1, lat2, lng2)

def generate_graph():
    """
    Genera el grafo de la ciudad de Cuenca con conexiones realistas
    Las distancias están en kilómetros
    """
    graph = {node: {} for node in CUENCA_NODES.keys()}
    
    # Definir conexiones viales con distancias aproximadas
    connections = [
        ('Parque Calderon', 'Catedral Nueva', 0.05),
        ('Parque Calderon', 'Plaza de las Flores', 0.10),
        ('Parque Calderon', 'Rio Tomebamba', 0.50),
        ('Catedral Nueva', 'Mercado 10 de Agosto', 0.30),
        ('Plaza de las Flores', 'Mercado 10 de Agosto', 0.25),
        ('Museo Pumapungo', 'Rio Tomebamba', 0.60),
        ('Museo Pumapungo', 'Barranco del Tomebamba', 0.35),
        ('Rio Tomebamba', 'Barranco del Tomebamba', 0.30),
        ('Barranco del Tomebamba', 'Mirador de Turi', 2.5),
        ('Universidad de Cuenca', 'Parque Calderon', 0.80),
        ('Universidad de Cuenca', 'Mercado 10 de Agosto', 0.60),
        ('Terminal Terrestre', 'Mirador de Turi', 1.8),
        ('Terminal Terrestre', 'Universidad de Cuenca', 2.0),
        ('Mirador de Turi', 'Museo Pumapungo', 2.8),
        ('Parque Calderon', 'Barranco del Tomebamba', 0.70),
        ('Plaza de las Flores', 'Rio Tomebamba', 0.45),
    ]
    
    # Crear grafo bidireccional
    for node1, node2, weight in connections:
        graph[node1][node2] = weight
        graph[node2][node1] = weight
    
    return graph

def astar_search(graph, start, goal):
    """
    Implementación del algoritmo A* para encontrar la ruta óptima
    
    Args:
        graph: Grafo representado como diccionario de adyacencias
        start: Nodo inicial
        goal: Nodo objetivo
    
    Returns:
        dict: Contiene path (ruta), cost (costo total), nodes_explored (nodos explorados)
    """
    # Conjunto de nodos a evaluar (open set)
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    # Diccionario para reconstruir el camino
    came_from = {}
    
    # g_score: costo del camino más barato desde start hasta cada nodo
    g_score = {node: float('inf') for node in graph.keys()}
    g_score[start] = 0
    
    # f_score: g_score + heurística (estimación del costo total)
    f_score = {node: float('inf') for node in graph.keys()}
    f_score[start] = calculate_distance(start, goal)
    
    # Nodos explorados para análisis
    nodes_explored = []
    
    while open_set:
        # Obtener el nodo con menor f_score
        current_f, current = heapq.heappop(open_set)
        nodes_explored.append(current)
        
        # Si llegamos al objetivo, reconstruir el camino
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            return {
                'path': path,
                'cost': g_score[goal],
                'nodes_explored': nodes_explored
            }
        
        # Evaluar vecinos
        for neighbor in graph[current]:
            # Calcular el g_score tentativo
            tentative_g_score = g_score[current] + graph[current][neighbor]
            
            # Si encontramos un mejor camino al vecino
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + calculate_distance(neighbor, goal)
                
                # Agregar vecino al open_set si no está
                if neighbor not in [node for _, node in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # No se encontró camino
    return {'path': [], 'cost': float('inf'), 'nodes_explored': nodes_explored}

def bfs_search(graph, start, goal):
    """Búsqueda en Anchura (BFS) - Algoritmo no informado"""
    from collections import deque
    
    queue = deque([(start, [start], 0)])
    visited = set([start])
    nodes_explored = []
    
    while queue:
        current, path, cost = queue.popleft()
        nodes_explored.append(current)
        
        if current == goal:
            return {
                'path': path,
                'cost': cost,
                'nodes_explored': nodes_explored
            }
        
        for neighbor in graph[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = path + [neighbor]
                new_cost = cost + graph[current][neighbor]
                queue.append((neighbor, new_path, new_cost))
    
    return {'path': [], 'cost': float('inf'), 'nodes_explored': nodes_explored}

def dfs_search(graph, start, goal):
    """Búsqueda en Profundidad (DFS) - Algoritmo no informado"""
    stack = [(start, [start], 0)]
    visited = set()
    nodes_explored = []
    best_path = None
    best_cost = float('inf')
    
    while stack:
        current, path, cost = stack.pop()
        
        if current not in visited:
            visited.add(current)
            nodes_explored.append(current)
            
            if current == goal and cost < best_cost:
                best_path = path
                best_cost = cost
            
            for neighbor in graph[current]:
                if neighbor not in visited:
                    new_path = path + [neighbor]
                    new_cost = cost + graph[current][neighbor]
                    stack.append((neighbor, new_path, new_cost))
    
    return {
        'path': best_path if best_path else [],
        'cost': best_cost,
        'nodes_explored': nodes_explored
    }

def create_map(graph, result, start_node, end_node):
    """
    Crea el mapa de Cuenca con Folium mostrando la ruta óptima
    """
    # Centro del mapa en Cuenca
    cuenca_center = [-2.9, -79.0]
    m = folium.Map(location=cuenca_center, zoom_start=13, tiles='OpenStreetMap')
    
    # Dibujar todas las conexiones del grafo (en gris)
    for node1, neighbors in graph.items():
        for node2 in neighbors:
            coords1 = [CUENCA_NODES[node1]['lat'], CUENCA_NODES[node1]['lng']]
            coords2 = [CUENCA_NODES[node2]['lat'], CUENCA_NODES[node2]['lng']]
            
            folium.PolyLine(
                locations=[coords1, coords2],
                color='gray',
                weight=2,
                opacity=0.4
            ).add_to(m)
    
    # Dibujar la ruta óptima (en azul)
    if result['path']:
        path_coords = [
            [CUENCA_NODES[node]['lat'], CUENCA_NODES[node]['lng']] 
            for node in result['path']
        ]
        folium.PolyLine(
            locations=path_coords,
            color='blue',
            weight=5,
            opacity=0.8,
            popup='Ruta Óptima'
        ).add_to(m)
    
    # Marcar nodos explorados (en amarillo)
    for node in result['nodes_explored']:
        if node not in [start_node, end_node] and node not in result['path']:
            folium.CircleMarker(
                location=[CUENCA_NODES[node]['lat'], CUENCA_NODES[node]['lng']],
                radius=6,
                color='orange',
                fill=True,
                fillColor='yellow',
                fillOpacity=0.6,
                popup=f"Explorado: {node}"
            ).add_to(m)
    
    # Marcar nodos de la ruta (en azul)
    for node in result['path']:
        if node not in [start_node, end_node]:
            folium.CircleMarker(
                location=[CUENCA_NODES[node]['lat'], CUENCA_NODES[node]['lng']],
                radius=7,
                color='darkblue',
                fill=True,
                fillColor='cyan',
                fillOpacity=0.8,
                popup=f"Ruta: {node}"
            ).add_to(m)
    
    # Marcar todos los nodos del grafo
    for node, coords in CUENCA_NODES.items():
        if node not in [start_node, end_node]:
            folium.Marker(
                location=[coords['lat'], coords['lng']],
                popup=node,
                tooltip=node,
                icon=folium.Icon(color='gray', icon='info-sign')
            ).add_to(m)
    
    # Marcar el inicio (verde)
    folium.Marker(
        location=[CUENCA_NODES[start_node]['lat'], CUENCA_NODES[start_node]['lng']],
        popup=f"Inicio: {start_node}",
        tooltip=start_node,
        icon=folium.Icon(color='green', icon='play')
    ).add_to(m)
    
    # Marcar el destino (rojo)
    folium.Marker(
        location=[CUENCA_NODES[end_node]['lat'], CUENCA_NODES[end_node]['lng']],
        popup=f"Destino: {end_node}",
        tooltip=end_node,
        icon=folium.Icon(color='red', icon='stop')
    ).add_to(m)
    
    return m

# ==================== INTERFAZ STREAMLIT ====================

st.title("🗺️ Sistema de Optimización de Rutas - Cuenca")
st.markdown("### Algoritmo A* y Comparación con Búsquedas No Informadas")

# Panel de información
with st.expander("📚 Modelado del Problema", expanded=True):
    st.markdown("""
    **Descripción del Modelado:**
    - **Nodos:** 10 puntos de interés representativos de Cuenca (Parque Calderón, Catedral Nueva, Mirador de Turi, etc.)
    - **Aristas:** Conexiones viales entre puntos de interés
    - **Pesos:** Distancias en kilómetros entre ubicaciones
    - **Heurística (A*):** Distancia euclidiana calculada con la fórmula de Haversine
    - **Función de costo:** f(n) = g(n) + h(n), donde:
        - g(n) = costo acumulado desde el inicio
        - h(n) = estimación heurística hasta el objetivo
    """)

# Generar el grafo
graph = generate_graph()

# Sidebar para controles
st.sidebar.header("⚙️ Configuración")

# Selección de algoritmo
algorithm = st.sidebar.selectbox(
    "Seleccionar Algoritmo",
    ["A* (Búsqueda Informada)", "BFS (Búsqueda en Anchura)", "DFS (Búsqueda en Profundidad)"]
)

# Selección de nodos
node_names = list(CUENCA_NODES.keys())
start_node = st.sidebar.selectbox("📍 Punto de Origen", node_names, index=0)
end_node = st.sidebar.selectbox("🎯 Punto de Destino", node_names, index=3)

# Botón para buscar ruta
if st.sidebar.button("🚀 Calcular Ruta Óptima", type="primary"):
    if start_node == end_node:
        st.sidebar.error("⚠️ El origen y destino deben ser diferentes")
    else:
        with st.spinner("Calculando ruta óptima..."):
            # Ejecutar el algoritmo seleccionado
            start_time = time.time()
            
            if "A*" in algorithm:
                result = astar_search(graph, start_node, end_node)
                algo_name = "A*"
            elif "BFS" in algorithm:
                result = bfs_search(graph, start_node, end_node)
                algo_name = "BFS"
            else:
                result = dfs_search(graph, start_node, end_node)
                algo_name = "DFS"
            
            execution_time = (time.time() - start_time) * 1000  # en milisegundos
            
            # Guardar en session_state
            st.session_state['result'] = result
            st.session_state['execution_time'] = execution_time
            st.session_state['algorithm'] = algo_name
            st.session_state['start_node'] = start_node
            st.session_state['end_node'] = end_node

# Mostrar resultados si existen
if 'result' in st.session_state:
    result = st.session_state['result']
    execution_time = st.session_state['execution_time']
    algo_name = st.session_state['algorithm']
    start_node = st.session_state['start_node']
    end_node = st.session_state['end_node']
    
    if result['path']:
        # Crear dos columnas
        col1, col2 = st.columns([2, 1])
        
        with col1:
            st.subheader(f"🗺️ Mapa de la Ruta - {algo_name}")
            # Crear y mostrar el mapa
            map_obj = create_map(graph, result, start_node, end_node)
            folium_static(map_obj, width=800, height=500)
        
        with col2:
            st.subheader("📊 Estadísticas")
            
            # Métricas
            st.metric("Distancia Total", f"{result['cost']:.2f} km")
            st.metric("Nodos Explorados", len(result['nodes_explored']))
            st.metric("Pasos en la Ruta", len(result['path']))
            st.metric("Tiempo de Ejecución", f"{execution_time:.3f} ms")
            
            # Detalles de la ruta
            st.subheader("🛣️ Ruta Detallada")
            for i, node in enumerate(result['path'], 1):
                if i < len(result['path']):
                    next_node = result['path'][i]
                    distance = graph[node][next_node]
                    st.write(f"{i}. **{node}** → ({distance:.2f} km)")
                else:
                    st.write(f"{i}. **{node}** 🎯")
        
        # Sección de análisis comparativo
        st.markdown("---")
        st.subheader("📈 Análisis Comparativo de Algoritmos")
        
        # Ejecutar los tres algoritmos para comparación
        with st.spinner("Ejecutando comparación de algoritmos..."):
            astar_result = astar_search(graph, start_node, end_node)
            bfs_result = bfs_search(graph, start_node, end_node)
            dfs_result = dfs_search(graph, start_node, end_node)
            
            comparison_data = {
                'Algoritmo': ['A*', 'BFS', 'DFS'],
                'Distancia (km)': [
                    f"{astar_result['cost']:.2f}",
                    f"{bfs_result['cost']:.2f}",
                    f"{dfs_result['cost']:.2f}"
                ],
                'Nodos Explorados': [
                    len(astar_result['nodes_explored']),
                    len(bfs_result['nodes_explored']),
                    len(dfs_result['nodes_explored'])
                ],
                'Pasos en Ruta': [
                    len(astar_result['path']),
                    len(bfs_result['path']),
                    len(dfs_result['path'])
                ]
            }
            
            df = pd.DataFrame(comparison_data)
            st.table(df)
        
        st.markdown("""
        **Conclusiones:**
        - **A*** utiliza una heurística (distancia euclidiana) para explorar menos nodos y encontrar la ruta óptima eficientemente
        - **BFS** garantiza encontrar la ruta óptima pero explora más nodos que A*
        - **DFS** puede no encontrar la ruta óptima y su eficiencia es variable
        - **Ventaja de A*:** Combina el costo real con la estimación heurística, logrando la mejor eficiencia
        """)
    else:
        st.error("❌ No se encontró una ruta entre los puntos seleccionados")
else:
    st.info("👈 Selecciona el origen, destino y algoritmo en el panel lateral, luego presiona 'Calcular Ruta Óptima'")

# Footer
st.markdown("---")
st.markdown("""
<div style='text-align: center'>
    <p>🎓 Proyecto: Optimización de Rutas con Algoritmo A*</p>
    <p>📍 Ciudad: Cuenca, Ecuador</p>
</div>
""", unsafe_allow_html=True)